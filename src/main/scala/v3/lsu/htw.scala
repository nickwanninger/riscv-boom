package boom.v3.lsu

import chisel3._
import chisel3.util._
import chisel3.experimental.SourceInfo

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.constants._
import freechips.rocketchip.rocket.HellaCacheIO

import boom.v3.common._
import freechips.rocketchip.rocket.PRV.U
import freechips.rocketchip.tilelink.TLMessages.c
import freechips.rocketchip.diplomacy.BufferParams.pipe

class HTE(implicit p: Parameters) extends BoomBundle()(p) {
  val try_phys = Bool()
  val reserved = UInt((64 - maxSVAddrBits - 1).W)
  val addr = UInt(maxSVAddrBits.W)
}

class L2HTLBEntry(nSets: Int)(implicit p: Parameters) extends BoomBundle()(p) {
  val idxBits = log2Ceil(nSets)
  val tagBits = handleBits - idxBits
  val tag = UInt(tagBits.W)
  val try_phys = Bool()
  val addr = UInt(maxSVAddrBits.W)
}

class HTWReq(implicit p: Parameters) extends BoomBundle()(p) {
  val hid = UInt(handleBits.W)
}

class HTWResp(implicit p: Parameters) extends BoomBundle()(p) {
  val hte = new HTE
}

class EvictionReq(implicit p: Parameters) extends BoomBundle()(p) {
  val addr = UInt(xLen.W)
  val try_phys = Bool()
  val phys = Bool()
  val hid = UInt(handleBits.W)
}

class HTLBHTWIO(implicit p: Parameters) extends BoomBundle()(p) {
  val req = Decoupled(Valid(new HTWReq))
  val resp = Flipped(Valid(new HTWResp))
  val evict = Decoupled(new EvictionReq)
  val evict_resp = Input(Bool())
  val l1_dumped = Output(Bool())
  val l1miss = Output(Bool())
}

class HTWPerfEvents(implicit p: Parameters) extends BoomBundle()(p) {
  val l2miss = Bool()
  val l1miss = Bool()
}

class DatapathHTWIO(implicit p: Parameters) extends BoomBundle()(p) {
  val perf = Output(new HTWPerfEvents())
  val customCSRs = Flipped(coreParams.customCSRs)
  val htDumped = Output(Bool())
  val htInvald = Output(Bool())
  val clock_enabled = Output(Bool())
  val clear_htlb = Input(Bool())
}

class HTW(implicit p: Parameters) extends BoomModule()(p) {
  require(maxSVAddrBits == 39)
  require(new HTE().getWidth == 64)
  val io = IO(new Bundle {
    val requestor = Flipped(new HTLBHTWIO)
    val mem = new HellaCacheIO
    val dpath = new DatapathHTWIO
    val ptw_access = Input(Bool())
  })
  io.dpath.customCSRs := DontCare

  // State Machine
  val s_ready :: s_req :: s_wait1 :: s_wait2 :: s_wait3 :: s_victim :: s_dumping :: s_dumping_wait :: s_invalidating :: s_invalidated :: Nil = Enum(10)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)
  val l2_refill_wire = Wire(Bool())
  state := Mux(io.dpath.customCSRs.htBase =/= 0.U, OptimizationBarrier(next_state), s_ready)

  val resp_valid = RegNext(RegInit(false.B))

  val clock_en =
    state =/= s_ready || l2_refill_wire || io.requestor.req.valid || io.dpath.customCSRs.disableDCacheClockGate
  io.dpath.clock_enabled := usingVM.B && clock_en
  val gated_clock =
    if (!usingVM || !tileParams.dcache.get.clockGate) clock
    else ClockGate(clock, clock_en, "htw_clock_gate")
  withClock(gated_clock) {
    val l2_refill = RegNext(false.B)
    l2_refill_wire := l2_refill

    io.requestor.req.ready := (state === s_ready) && !l2_refill_wire

    val r_hte = Reg(new HTE)
    val v_hte = Reg(new HTE)
    val v_hid = Reg(UInt(handleBits.W))

    /* debug print for handle table walks */
    when(io.mem.req.valid && state =/= s_dumping_wait) {
      midas.targetutils.SynthesizePrintf(printf(
        "[HTW] -> [Mem] Looking up HID: %d at %x\n",
        (io.mem.req.bits.addr - io.dpath.customCSRs.htBase)/8.U(xLen.W),
        io.mem.req.bits.addr
      ))
    }

    // Handle HTW Responses
    val mem_resp_valid = RegNext(io.mem.resp.valid)
    val mem_resp_data = RegNext(io.mem.resp.bits.data)
    io.mem.uncached_resp.map { resp =>
      assert(!(resp.valid && io.mem.resp.valid))
      resp.ready := true.B
      when(resp.valid) {
        mem_resp_valid := true.B
        mem_resp_data := resp.bits.data
      }
    }

    // Send completed request to HTLB
    // NOTE: we assume that the HTE will always be valid
    io.requestor.resp.valid := resp_valid
    io.requestor.resp.bits.hte := r_hte

    /* debug print for response from HTW */
    when(io.requestor.resp.valid) {
      midas.targetutils.SynthesizePrintf(printf(
        "[HTW] -> [HTLB] Found HID %d to be %x\n",
        io.requestor.req.bits.bits.hid,
        io.requestor.resp.bits.hte.addr
      ))
    }

    // Victim Cache Logic
    io.requestor.evict.ready := state === s_ready
    when(state === s_victim) {
      midas.targetutils.SynthesizePrintf(printf("[HTW] Inserting Victim Entry: %d - %x\n", v_hid, v_hte.addr))
    }
    l2_refill := state === s_victim

    val nL2HTLBSets = boomParams.nL2HTLBEntries / boomParams.nL2HTLBWays
    val idxBits = log2Ceil(nL2HTLBSets)
    val d_ways = RegInit(0.U(log2Ceil(boomParams.nL2HTLBWays).W))
    val d_hid = RegInit(0.U(handleBits.W))
    val dumped_htlb_idx = RegInit(0.U((log2Ceil(boomParams.nL2HTLBEntries) + 1).W))

    val (l2_hit, l2_error, l2_hte, l2_htlb_ram) = {
      val code = new ParityCode
      require(isPow2(boomParams.nL2HTLBEntries))
      require(isPow2(boomParams.nL2HTLBWays))
      require(boomParams.nL2HTLBEntries >= boomParams.nL2HTLBWays)
      require(isPow2(nL2HTLBSets))

      val l2_plru = new SetAssocLRU(nL2HTLBSets, boomParams.nL2HTLBWays, "plru")

      val ram = DescribedSRAM(
        name = "l2_htlb_ram",
        desc = "L2 HTLB",
        size = nL2HTLBSets,
        data = Vec(
          boomParams.nL2HTLBWays,
          UInt(code.width(new L2HTLBEntry(nL2HTLBSets).getWidth).W)
        )
      )

      val valid = RegInit(
        VecInit(Seq.fill(boomParams.nL2HTLBWays)(0.U(nL2HTLBSets.W)))
      )
      // use r_req to construct tag
      val (r_tag, r_idx) = Split(io.requestor.req.bits.bits.hid, idxBits)

      /** the valid vec for the selected set(including n ways) */
      val r_valid_vec = valid.map(_(r_idx)).asUInt
      val r_valid_vec_q = Reg(UInt(boomParams.nL2HTLBWays.W))
      r_valid_vec_q := r_valid_vec
      when(l2_refill) {
        val (v_tag, v_idx) = Split(v_hid, idxBits)
        val refill_r_valid_vec = valid.map(_(v_idx)).asUInt

        val refill_s0_valid = true.B
        val refill_s1_valid = RegNext(refill_s0_valid)
        val refill_s2_valid = RegNext(refill_s1_valid)
        // read from htlb idx
        val refill_s1_rdata = ram.read(v_idx, refill_s0_valid)
        val refill_s2_rdata =
          refill_s1_rdata.map(refill_s1_rdway => code.decode(RegEnable(refill_s1_rdway, refill_s1_valid)))
        val refill_s2_valid_vec = RegEnable(refill_r_valid_vec, refill_s1_valid)
        val refill_s2_error = (0 until boomParams.nL2HTLBWays)
          .map(way => refill_s2_valid_vec(way) && refill_s2_rdata(way).error)
          .orR
        when(refill_s2_valid && refill_s2_error) { valid.foreach { _ := 0.U } }
        printf("s2_valid: %d, s2_error: %d\n", refill_s2_valid, refill_s2_error)
        // decode
        val refill_s2_entry_vec =
          refill_s2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(nL2HTLBSets)))
        val refill_s2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
          refill_s2_valid_vec(way) && (v_tag === refill_s2_entry_vec(way).tag)
        )
        // printf("r_idx: %x, r_tag: %x, entry-vec-addr: %x\n", r_idx, r_tag, s2_entry_vec(0).addr)
        val refill_s2_hit = refill_s2_valid && refill_s2_hit_vec.orR
        when (refill_s2_valid) {
          printf("refill_s2_hit: %x on set: %d, tag: %d\n", refill_s2_hit_vec.asUInt, v_idx, v_tag)
        }

        when (refill_s2_valid && !refill_s2_hit_vec.orR) {
          val v_valid_vec = valid.map(_(v_idx)).asUInt
          val v_valid_vec_q = Reg(UInt(boomParams.nL2HTLBWays.W))
        
          // replacement way
          val v_l2_plru_way = Reg(UInt(log2Ceil(boomParams.nL2HTLBWays max 1).W))
          v_valid_vec_q := v_valid_vec
          v_l2_plru_way := (if (boomParams.nL2HTLBWays > 1) l2_plru.way(v_idx)
                            else 0.U)

          val entry = Wire(new L2HTLBEntry(nL2HTLBSets))
          entry.try_phys := v_hte.try_phys
          entry.addr := v_hte.addr
          entry.tag := v_tag
          // if all the way are valid, use plru to select one way to be replaced,
          // otherwise use PriorityEncoderOH to select one
          val wmask =
            if (boomParams.nL2HTLBWays > 1)
              Mux(
                v_valid_vec_q.andR,
                UIntToOH(v_l2_plru_way, boomParams.nL2HTLBWays),
                PriorityEncoderOH(~v_valid_vec_q)
              )
            else 1.U(1.W)
          ram.write(
            v_idx,
            VecInit(Seq.fill(boomParams.nL2HTLBWays)(code.encode(entry.asUInt))),
            wmask.asBools
          )
          midas.targetutils.SynthesizePrintf(printf("[HTW] Inserting with addr: %x into set %d, way (%x) %d (tag)\n", entry.addr, v_idx, wmask, v_tag))

          val mask = UIntToOH(v_idx)
          printf("Mask: %x\n", mask)
          for (way <- 0 until boomParams.nL2HTLBWays) {
            when(wmask(way)) {
              valid(way) := valid(way) | mask
            }
          }
        }
      }

      val s0_valid = !l2_refill
      val s1_valid = RegNext(s0_valid && io.requestor.req.valid)
      val s2_valid = RegNext(s1_valid)
      // read from htlb idx
      val s1_rdata = ram.read(r_idx, s0_valid)
      val s2_rdata =
        s1_rdata.map(s1_rdway => code.decode(RegEnable(s1_rdway, s1_valid)))
      val s2_valid_vec = RegEnable(r_valid_vec, s1_valid)
      val s2_error = (0 until boomParams.nL2HTLBWays)
        .map(way => s2_valid_vec(way) && s2_rdata(way).error)
        .orR
      when(s2_valid && s2_error) { valid.foreach { _ := 0.U } }
      // printf("s2_valid: %d, s2_error: %d\n", s2_valid, s2_error)
      // decode
      val s2_entry_vec =
        s2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(nL2HTLBSets)))
      val s2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
        s2_valid_vec(way) && (r_tag === s2_entry_vec(way).tag)
      )
      // printf("r_idx: %x, r_tag: %x, entry-vec-addr: %x\n", r_idx, r_tag, s2_entry_vec(0).addr)
      val s2_hit = s2_valid && s2_hit_vec.orR
      io.dpath.perf.l2miss := s2_valid && !(s2_hit_vec.orR)
      when(io.dpath.perf.l2miss) {
        midas.targetutils.SynthesizePrintf(printf("[HTW] L2 Miss\n"))
      }
      when(s2_hit) {
        printf("[HTW] s2_hit: %b\n", s2_hit_vec.asUInt)
        // val invl_mask = UIntToOH(r_idx)
        // for (way <- 0 until boomParams.nL2HTLBWays) {
        //     valid(way) := valid(way) & ~invl_mask
        // }
        assert((PopCount(s2_hit_vec) === 1.U) || s2_error, "L2 HTLB multi-hit")
      }

      val s2_hte = Wire(new HTE)
      val s2_hit_entry = Mux1H(s2_hit_vec, s2_entry_vec)
      s2_hte.addr := s2_hit_entry.addr
      s2_hte.reserved := 0.U
      s2_hte.try_phys := s2_hit_entry.try_phys

      for (way <- 0 until boomParams.nL2HTLBWays) {
        ccover(
          s2_hit && s2_hit_vec(way),
          s"L2_HTLB_HIT_WAY$way",
          s"L2 HTLB hit way$way"
        )
      }

      when(s2_hit) {
        printf("[HTW] Hit with addr: %x (%d)\n", s2_hit_entry.addr, s2_hit_entry.tag)
      }

      val set_idx = RegInit(0.U((idxBits + 1).W))
      val way_idx = RegInit(0.U((log2Ceil(boomParams.nL2HTLBWays) + 1).W))
      io.dpath.htDumped := set_idx === nL2HTLBSets.U && state === s_dumping
      io.dpath.htInvald := state === s_invalidated

      when (io.dpath.htDumped) {
        midas.targetutils.SynthesizePrintf(printf("[HTW] Finished dumping\n"))
        dumped_htlb_idx := 0.U
      }

      when (io.dpath.htInvald) {
        midas.targetutils.SynthesizePrintf(printf("[HTW] Finished invalidating\n"))
      }

      when(io.requestor.l1_dumped && state === s_ready && io.dpath.customCSRs.htDump.orR) {
        midas.targetutils.SynthesizePrintf(printf("[HTW] Starting to dump L2\n"))
      }

      when (!io.dpath.customCSRs.htDump.orR && state === s_dumping) {
        set_idx := 0.U
        way_idx := 0.U
        next_state := s_ready
        printf("[HTW] Finished dumping all entries\n")
      }

      // for (s <- 0 until boomParams.nL2HTLBWays) {
      //   printf("Valid(%d): %b\n", s.U, valid(s).asUInt)
      // }

      val pipeline_stage = RegInit(0.U(2.W))

      val dr_valid_vec = valid.map(_(set_idx(idxBits-1,0))).asUInt
      val ds0_valid = state === s_dumping && pipeline_stage === 0.U
      val ds1_valid = RegNext(ds0_valid)
      val ds2_valid = RegNext(ds1_valid) && pipeline_stage === 2.U
      // read from tlb idx
      val ds1_rdata = ram.read(set_idx(idxBits-1,0), ds0_valid)
      val ds2_rdata =
        ds1_rdata.map(ds1_rdway => code.decode(RegEnable(ds1_rdway, ds1_valid)))
      val ds2_error = (0 until boomParams.nL2HTLBWays)
        .map(way => dr_valid_vec(way) && ds2_rdata(way).error)
        .orR
      when(ds2_valid && ds2_error) { valid.foreach { _ := 0.U } }
      // printf("ds2_valid: %d, ds2_error: %d\n", ds2_valid, ds2_error)
      // decode
      val ds2_entry_vec =
        ds2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(nL2HTLBSets)))

      // TODO: reading old ds2_entry_vec, before new one comes in.
      // it is there for 3 cycles before the real one comes in. do we not restart the reading clock?
      // printf("ds2_entry_vec: %x\n", ds2_entry_vec(0).addr)

      val ds2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
        dr_valid_vec(way)
      )

      val ds2_hit = ds2_valid && ds2_hit_vec.orR && !ds2_error
      // val ds2_hit = ds2_valid && ds2_hit_vec.orR && !ds2_error

      val ds2_hte = Wire(new L2HTLBEntry(nL2HTLBSets))

      val way_idx_update = ((state === s_dumping && !ds2_hit && pipeline_stage === 3.U) || (state === s_dumping_wait && mem_resp_valid))
      val set_idx_update = (way_idx === (boomParams.nL2HTLBWays- 1).U) && (way_idx_update)

      val way_clear = (set_idx_update || (state === s_dumping_wait && way_idx === boomParams.nL2HTLBWays.U)) || (!io.dpath.customCSRs.htDump.orR && state === s_dumping)
      val set_clear = !io.dpath.customCSRs.htDump.orR && state === s_dumping

      val is_dumping = state === s_dumping && way_idx < boomParams.nL2HTLBWays.U && set_idx < nL2HTLBSets.U
      val way = RegInit(0.U(log2Ceil(boomParams.nL2HTLBWays).W))
      when ((way_idx_update && !set_idx_update) || RegNext(way_idx_update && set_idx_update)) {
        val new_way = l2_plru.way(set_idx)
        // printf("Updating way: %d -> new_way: %d, with set %d\n", way, new_way, set_idx)
        way := l2_plru.way(set_idx)
      }

      when (is_dumping) {
        val ds2_hit_entry = Mux1H(UIntToOH(way), ds2_entry_vec)
        ds2_hte.addr := ds2_hit_entry.addr
        ds2_hte.tag := ds2_hit_entry.tag
        ds2_hte.try_phys := DontCare

        when(ds2_hit && dr_valid_vec(way)) {
          d_hid := Cat(Mux1H(UIntToOH(way), ds2_entry_vec).tag, set_idx(idxBits-1,0))
          midas.targetutils.SynthesizePrintf(printf(
            "[HTW]  L2Entry: %d: %d, Valid(%d) %x - %x\n",
            set_idx(idxBits-1,0), 
            way,
            dr_valid_vec(way),
            Cat(Mux1H(UIntToOH(way), ds2_entry_vec).tag, set_idx(idxBits-1,0)),
            ds2_hte.addr
          ))
          next_state := s_dumping_wait
          l2_plru.access(set_idx(idxBits-1,0), way)
          // printf("1 Accessing set: %d, way: %d\n", set_idx(idxBits-1,0), way)
        } .elsewhen(ds2_valid) {
          // printf("2 Accessing set: %d, way: %d\n", set_idx(idxBits-1,0), way)
          l2_plru.access(set_idx(idxBits-1,0), way)
        }
        // printf("Set: %d, Way: %d - Valid(%d), ds2(%d)\n", set_idx, way, dr_valid_vec(way), ds2_valid)
      }.otherwise {
        ds2_hte := DontCare
      }

      way_idx := Mux(way_clear, 0.U, Mux(way_idx_update, way_idx + 1.U, way_idx))

      // Update pipeline stage
      when ((pipeline_stage >= 2.U && way_idx_update) || (set_clear && way_clear)) {
        pipeline_stage := 0.U
      } .elsewhen(state === s_dumping) {
        // pipeline_stage := Mux(pipeline_stage === 2.U, 2.U, pipeline_stage + 1.U)
        pipeline_stage := pipeline_stage + 1.U
      }

      set_idx := Mux(set_idx_update, 
                     set_idx + 1.U, 
                     Mux(state === s_dumping_wait && way_idx === boomParams.nL2HTLBWays.U, 
                         d_hid(idxBits-1,0) + 1.U, 
                         Mux(set_clear, 0.U,
                         set_idx)))

      // printf("SetIdx: %d, WayIdx: %d\n", set_idx, way_idx)
      // printf("SetIdxUpdate: %d, WayIdxUpdate: %d\n", set_idx_update, way_idx_update)
      // printf("Pipeline Stage: %d\n", pipeline_stage)
      // printf("WayClear: %d, SetClear: %d\n", way_clear, set_clear)

      // when (state === s_invalidating) {
      when (io.dpath.customCSRs.htInval.orR) {
        when (io.dpath.customCSRs.htInval(handleBits -1, 0) === ((BigInt(1) << handleBits) - 1).U) {
          midas.targetutils.SynthesizePrintf(printf("[HTW] Invalidating all entries\n"))
          for (way <- 0 until boomParams.nL2HTLBWays) {
            valid(way) := 0.U
          }
          next_state := s_invalidated
        } .otherwise {
          midas.targetutils.SynthesizePrintf(printf("[HTW] Invalidating %x\n", io.dpath.customCSRs.htInval(handleBits -1, 0)))
          val (i_tag, i_idx) = Split(io.dpath.customCSRs.htInval(handleBits -1, 0), idxBits)

          val i_valid_vec = valid.map(_(i_idx)).asUInt
          val i0_valid = state === s_invalidating
          val i1_valid = RegNext(i0_valid)
          val i2_valid = RegNext(i1_valid)
          // read from tlb idx
          val i1_rdata = ram.read(i_idx, i0_valid)
          val i2_rdata =
            i1_rdata.map(i1_rdway => code.decode(RegEnable(i1_rdway, i1_valid)))
          val i2_valid_vec = RegEnable(i_valid_vec, i1_valid)
          // val i2_g_vec = RegEnable(VecInit(g.map(_(r_idx))), i1_valid)
          val i2_error = (0 until boomParams.nL2HTLBWays)
            .map(way => i2_valid_vec(way) && i2_rdata(way).error)
            .orR
          when(i2_valid && i2_error) { valid.foreach { _ := 0.U } }
          // printf("i2_valid: %d, i2_error: %d\n", i2_valid, i2_error)
          // decode
          val i2_entry_vec =
            i2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(nL2HTLBSets)))
          val i2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
            i2_valid_vec(way) && (i_tag === i2_entry_vec(way).tag)
          )
          // printf("r_idx: %x, r_tag: %x, entry-vec-addr: %x\n", r_idx, r_tag, i2_entry_vec(0).addr)
          val i2_hit = i2_valid && i2_hit_vec.orR

          when(i2_hit) {
            val invl_mask = UIntToOH(i_idx)
            // printf("Invl Mask: %x\n", invl_mask)
            for (way <- 0 until boomParams.nL2HTLBWays) {
                valid(way) := valid(way) & ~invl_mask
                //   g(way) := Mux(r_pte.g, g(way) | mask, g(way) & ~mask)
            }
            // printf("Valid: %x\n", valid(0).asUInt)
            assert((PopCount(i2_hit_vec) === 1.U) || i2_error, "L2 HTLB multi-hit")
          }

          next_state := Mux(i2_valid, s_invalidated, s_invalidating)
        }
      }
    
      when (reset.asBool || io.dpath.clear_htlb) {
        when (io.dpath.clear_htlb) {
          midas.targetutils.SynthesizePrintf(printf("[HTW] Clearing HTLB\n"))
        }
        for (way <- 0 until boomParams.nL2HTLBWays) {
          valid(way) := 0.U
        }
      }

      (s2_hit, s2_error, s2_hte, Some(ram))
    }
    // printf("%d, %d, %d\n", l2_hit, l2_error, mem_resp_valid)

    // Debug HTW response
    val tmp = mem_resp_data.asTypeOf(new HTE())
    val pte = WireDefault(tmp)
    pte.try_phys := true.B
    when(mem_resp_valid && state === s_wait3) {
      midas.targetutils.SynthesizePrintf(printf(
        "[HTW] Found HTE - Frozen: Reserved: %x, Addr: %x, try_phys: %d\n",
        // pte.frozen,
        pte.reserved,
        pte.addr,
        pte.try_phys
      ))
    }

    r_hte := OptimizationBarrier(
      Mux(l2_hit && !l2_error, l2_hte, Mux(mem_resp_valid, pte, r_hte))
    )

    // Finite State Machine Logic
    switch(state) {
      is(s_ready) {
        next_state := Mux(
          io.requestor.req.valid,
          s_req,
          Mux(io.requestor.evict.valid, s_victim, 
          Mux(io.requestor.l1_dumped && state =/= s_dumping_wait, s_dumping,
          Mux(io.dpath.customCSRs.htInval.orR, s_invalidating, 
          s_ready)
        )))

        v_hid := Mux(io.requestor.evict.valid, io.requestor.evict.bits.hid, 0.U)
        v_hte.addr := Mux(io.requestor.evict.valid, io.requestor.evict.bits.addr, 0.U)
        v_hte.try_phys := Mux(io.requestor.evict.valid, io.requestor.evict.bits.try_phys, false.B)
        v_hte.reserved := Mux(io.requestor.evict.valid, 0.U, 0.U)
      }
      is(s_req) {
        next_state := Mux(io.mem.req.ready, s_wait1, s_req)
      }
      is(s_wait1) {
        next_state := Mux(l2_hit, s_req, s_wait2)
      }
      is (s_wait2) {
        next_state := s_wait3
      }
      is(s_victim) {
        next_state := ShiftRegister(s_ready, 3)
      }
      is(s_invalidated) {
        next_state := s_ready
      }
    }

    // Hit Handling Logic or HTW response logic
    when(l2_hit && !l2_error) {
      // assert(state === s_req || state === s_wait1)
      next_state := s_ready
      resp_valid := true.B
    }

    // printf("State: %d\n", state)
    // when (mem_resp_valid && state =/= s_dumping_wait) {
    when (mem_resp_valid && (state =/= s_dumping_wait && state =/= s_dumping)) {
      // assert(state === s_wait3)
      next_state := s_ready
      resp_valid := true.B
    }

    when (io.mem.s2_nack && (state === s_wait2 || state === s_dumping_wait)) {
      next_state := Mux(state === s_wait2, s_req, s_dumping_wait)
    }

    io.requestor.evict_resp := l2_refill

    io.dpath.perf.l1miss := io.requestor.l1miss

    // HT Lookup
    val hte_vaddr =
      io.dpath.customCSRs.htBase + io.requestor.req.bits.bits.hid * ((new HTE().getWidth.U) / 8.U((log2Ceil(new HTE().getWidth) + 1).W))

    val d_hte_vaddr = io.dpath.customCSRs.htDump + (dcacheParams.nTLBSets*dcacheParams.nTLBWays*8).U + dumped_htlb_idx*8.U

    when (state === s_dumping_wait) {
      midas.targetutils.SynthesizePrintf(printf("[HTW] Dumping hid %x to %x (mem_resp_valid: %d)\n", d_hid, d_hte_vaddr, mem_resp_valid))
    }

    when (state === s_dumping_wait && mem_resp_valid) {
      midas.targetutils.SynthesizePrintf(printf("[HTW] Finished dumping %d-th hid %x\n", dumped_htlb_idx, d_hid))
      dumped_htlb_idx := dumped_htlb_idx + 1.U
      next_state := s_dumping
    }

    // Prepare Memory Request
    io.mem.keep_clock_enabled := false.B

    io.mem.req.valid := state === s_req  || (state === s_dumping_wait && !mem_resp_valid)
    io.mem.req.bits.phys := false.B
    io.mem.req.bits.cmd := Mux(state === s_dumping_wait, M_XWR, M_XRD)
    io.mem.req.bits.size :=
    log2Ceil(
      xLen / 8
    ).U // TODO: confirm this makes sense
    io.mem.req.bits.signed := false.B
    io.mem.req.bits.addr := Mux(state === s_dumping_wait, d_hte_vaddr, hte_vaddr)
    io.mem.req.bits.idx.foreach(_ := Mux(state === s_dumping_wait, d_hte_vaddr, hte_vaddr)) // TODO: huh?
    io.mem.req.bits.dprv := PRV.S.U // HTW accesses are S-mode by definition
    io.mem.req.bits.dv := false.B
    io.mem.req.bits.tag := DontCare
    io.mem.req.bits.no_resp := false.B
    io.mem.req.bits.no_alloc := DontCare
    io.mem.req.bits.no_xcpt := DontCare
    io.mem.req.bits.data := Mux(state === s_dumping_wait, d_hid, 0.U)
    io.mem.req.bits.mask := Mux(state === s_dumping_wait, ((1 << coreDataBytes) - 1).U, 0.U)

    val replay_htw_req = io.ptw_access && io.mem.req.valid
    when (replay_htw_req) {
      midas.targetutils.SynthesizePrintf(printf("[HTW] Replaying HTW Access\n"))
    }

    // TODO: This may need to change if we get an exception in the middle of a handle table walk
    io.mem.s1_kill := l2_hit || (state =/= s_wait1 && state =/= s_dumping_wait) || replay_htw_req 
    io.mem.s1_data.data := Mux(state === s_dumping_wait, d_hid, 0.U)
    io.mem.s1_data.mask := Mux(state === s_dumping_wait, ((1 << coreDataBytes) - 1).U, 0.U)
    io.mem.s2_kill := Mux(state === s_dumping_wait, replay_htw_req, false.B)

    when (io.mem.req.valid) {
      midas.targetutils.SynthesizePrintf(printf("[HTW] Killed %d, %d\n", io.mem.s1_kill, io.mem.s2_kill))
    }

    // when (io.mem.s1_kill && io.mem.req.valid) {
      // printf("acccidentally killed? - %d, %d\n", l2_hit, state)
    // }
  }

  private def ccover(cond: Bool, label: String, desc: String)(implicit
      sourceInfo: SourceInfo
  ) =
    if (usingVM) property.cover(cond, s"HTW_$label", "MemorySystem;;" + desc)
}
