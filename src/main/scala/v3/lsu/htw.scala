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
  val phys = Bool()
  val try_phys = Bool()
  val ae = Bool()
  val reserved = UInt((64 - maxSVAddrBits - 3).W)
  val addr = UInt(maxSVAddrBits.W)
}

class L2HTLBEntry(nSets: Int)(implicit p: Parameters) extends BoomBundle()(p) {
  val idxBits = log2Ceil(nSets)
  val tagBits = handleBits - idxBits
  val tag = UInt(tagBits.W)
  val try_phys = Bool()
  val phys = Bool()
  val addr = UInt(maxSVAddrBits.W)
  val ae = Bool()
}

class HTWReq(implicit p: Parameters) extends BoomBundle()(p) {
  val hid = UInt(handleBits.W)
}

class HTWResp(implicit p: Parameters) extends BoomBundle()(p) {
  val hte = new HTE
}

class EvictionReq(implicit p: Parameters) extends BoomBundle()(p) {
  val addr = UInt(maxSVAddrBits.W)
  val try_phys = Bool()
  val phys = Bool()
  val ae = Bool()
  val hid = UInt(handleBits.W)
}

class HTLBHTWIO(implicit p: Parameters) extends BoomBundle()(p) {
  val req = Decoupled(Valid(new HTWReq))
  val resp = Flipped(Valid(new HTWResp))
  val evict = Decoupled(new EvictionReq)
  val evict_resp = Input(Bool())
  val l1_dumped = Output(Bool())
  val l1miss = Output(Bool())
  val htlb_enabled = Output(Bool())
  val pht_enabled = Output(Bool())
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

class TopLevelCacheEntry(implicit p: Parameters) extends BoomBundle()(p) {
  val valid = Bool()
  val tag = UInt((handleBits - 18).W)  // Since ind1 is 18 bits
  val data = UInt(maxSVAddrBits.W)     // Store the base address for inner walks
}

class HTW(implicit p: Parameters) extends BoomModule()(p) {
  require(maxSVAddrBits == 39)
  require(new HTE().getWidth == 64)
  val io = IO(new Bundle {
    val requestor = Flipped(new HTLBHTWIO)
    val mem = new HellaCacheIO
    val dpath = new DatapathHTWIO
    val ptw_done = Input(Bool())
  })
  io.dpath.customCSRs := DontCare
  io.dpath.perf.l1miss := io.requestor.l1miss

  // State Machine
  val s_ready :: s_req :: s_wait1 :: s_wait2 :: s_wait3 :: s_req2 :: s_wait4 :: s_wait5 :: s_wait6 :: s_victim1 :: s_victim2 :: s_victim3 :: s_dump :: s_dump_req :: s_dump_wait :: s_invalidating :: s_invalidated :: s_htw_replay_pending :: s_dump_replay_pending :: Nil = Enum(19)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)
  val l2_refill_wire = Wire(Bool())
  state := Mux(io.requestor.htlb_enabled, OptimizationBarrier(next_state), s_ready)

  val htBase = io.dpath.customCSRs.htBase(62, 0) // de-chicken bit the htBase CSR

  val resp_valid = RegNext(RegInit(false.B))

  when (io.requestor.htlb_enabled && state =/= next_state) {
    midas.targetutils.SynthesizePrintf(printf("[H2%d,%d\n", state, next_state))
  }

  val clock_en =
    state =/= s_ready || l2_refill_wire || io.requestor.req.valid || io.dpath.customCSRs.disableDCacheClockGate
  io.dpath.clock_enabled := usingVM.B && clock_en
  val gated_clock =
    if (!usingVM || !tileParams.dcache.get.clockGate) clock
    else ClockGate(clock, clock_en, "htw_clock_gate")
  withClock(gated_clock) {
    val found_hte = Reg(new HTE)
    val victim_hte = Reg(new EvictionReq)

    val l2_refill = RegNext(false.B)
    l2_refill_wire := l2_refill
    l2_refill := state === s_victim1 || state === s_victim2 || state === s_victim3

    io.requestor.req.ready := (state === s_ready) && !l2_refill_wire

    // Victim Cache Logic
    io.requestor.evict.ready := state === s_ready
    io.requestor.evict_resp := state === s_victim3 && next_state === s_ready

    // Handle HTW Responses
    val mem_resp_valid = RegNext(io.mem.resp.valid || Mux(io.requestor.pht_enabled, false.B, io.mem.s2_xcpt.asUInt =/= 0.U))
    val mem_resp_data = RegNext(io.mem.resp.bits.data)
    // io.mem.uncached_resp.map { resp =>
    //   assert(!(resp.valid && io.mem.resp.valid))
    //   resp.ready := true.B
    //   when(resp.valid) {
    //     mem_resp_valid := true.B
    //     mem_resp_data := resp.bits.data
    //   }
    // }

    // Send completed request to HTLB
    // NOTE: we assume that the HTE will always be valid
    io.requestor.resp.valid := resp_valid
    io.requestor.resp.bits.hte := found_hte

    val idxBits = log2Ceil(boomParams.nL2HTLBSets)
    val ways_dumped = RegInit(0.U(log2Ceil(boomParams.nL2HTLBWays).W))
    val hid_to_dump = RegInit(0.U(handleBits.W))
    val dumped_entry_idx = RegInit(0.U((log2Ceil(boomParams.nL2HTLBSets * boomParams.nL2HTLBWays) + 1).W))
    val set_idx = RegInit(0.U((idxBits + 1).W))
    val way_idx = RegInit(0.U((log2Ceil(boomParams.nL2HTLBWays) + 1).W))
    
    val hid = io.requestor.req.bits.bits.hid

    val entries_per_ht_bits = 18  // log2(4096 * 512 / 8) = log2(262144)
    val inner_walk_base = RegInit(0.U(xLen.W))

    // val top_level_cache_size = 16
    // def getIndex(hid: UInt) = (hid >> entries_per_ht_bits)(log2Ceil(top_level_cache_size)-1, 0)    // Bottom 4 bits of ind0 for 16 entries
    // def getTag(hid: UInt) = (hid >> entries_per_ht_bits)(handleBits-19, 4) // log2Ceil(top_level_cache_size))  // Remaining bits of ind0
    // def getInnerIndex(hid: UInt) = hid(entries_per_ht_bits-1, 0)  // Bottom 18 bits

    // val top_level_cache = if (boomParams.enableTwoStageHTW && false) {
    //   val cache = RegInit(VecInit(Seq.fill(16)(0.U.asTypeOf(new TopLevelCacheEntry))))

    //   val cache_lookup_idx = getIndex(hid)
    //   val cache_lookup_tag = getTag(hid)
    //   val inner_index = getInnerIndex(hid)
    //   val cache_entry = cache(cache_lookup_idx)
    //   val cache_hit = cache_entry.valid && cache_entry.tag === cache_lookup_tag

    //   when (state === s_wait3 && mem_resp_valid) {
    //     val fill_idx = getIndex(hid)
    //     cache(fill_idx).valid := true.B
    //     cache(fill_idx).tag := getTag(hid)
    //     cache(fill_idx).data := mem_resp_data

    //     midas.targetutils.SynthesizePrintf(printf(
    //       "[HTW] Cache fill: ind0=%d (idx=%d tag=%x) ind1=%d data=%x\n",
    //       hid >> entries_per_ht_bits, fill_idx, getTag(hid), inner_index, mem_resp_data
    //     ))
    //   }

    //   when (io.dpath.clear_htlb) {
    //     cache.foreach(_.valid := false.B)
    //   }

    //   Some((cache, inner_index))
    // } else None

    val (l2_hit, l2_error, l2_hte, l2_htlb_ram) = {
      val code = new ParityCode
      require(isPow2(boomParams.nL2HTLBSets * boomParams.nL2HTLBWays))
      require(isPow2(boomParams.nL2HTLBWays))
      require(boomParams.nL2HTLBSets * boomParams.nL2HTLBWays >= boomParams.nL2HTLBWays)
      require(isPow2(boomParams.nL2HTLBSets))

      val l2_plru = new SetAssocLRU(boomParams.nL2HTLBSets, boomParams.nL2HTLBWays, "plru")

      val ram = DescribedSRAM(
        name = "l2_htlb_ram",
        desc = "L2 HTLB",
        size = boomParams.nL2HTLBSets,
        data = Vec(
          boomParams.nL2HTLBWays,
          UInt(code.width(new L2HTLBEntry(boomParams.nL2HTLBSets).getWidth).W)
        )
      )

      val valid = RegInit(
        VecInit(Seq.fill(boomParams.nL2HTLBWays)(0.U(boomParams.nL2HTLBSets.W)))
      )
      // use r_req to construct tag
      val (r_tag, r_idx) = Split(hid, idxBits)

      /** the valid vec for the selected set(including n ways) */
      val r_valid_vec = valid.map(_(r_idx)).asUInt
      val r_valid_vec_q = Reg(UInt(boomParams.nL2HTLBWays.W))
      r_valid_vec_q := r_valid_vec
      when(l2_refill) {
        val (v_tag, v_idx) = Split(victim_hte.hid, idxBits)
        val refill_r_valid_vec = valid.map(_(v_idx)).asUInt

        val refill_s0_valid = l2_refill
        val refill_s1_valid = RegNext(refill_s0_valid)
        val refill_s2_valid = RegNext(refill_s1_valid)

        val refill_s1_rdata = ram.read(v_idx, refill_s0_valid)
        val refill_s2_rdata =
          refill_s1_rdata.map(refill_s1_rdway => code.decode(RegEnable(refill_s1_rdway, refill_s1_valid)))

        val refill_s2_valid_vec = RegEnable(refill_r_valid_vec, refill_s1_valid)

        val refill_s2_error = (0 until boomParams.nL2HTLBWays)
          .map(way => refill_s2_valid_vec(way) && refill_s2_rdata(way).error)
          .orR
        when(refill_s2_valid && refill_s2_error) { valid.foreach { _ := 0.U } }
        // printf("s2_valid: %d, s2_error: %d\n", refill_s2_valid, refill_s2_error)

        val refill_s2_entry_vec =
          refill_s2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(boomParams.nL2HTLBSets)))
        val refill_s2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
          refill_s2_valid_vec(way) && (v_tag === refill_s2_entry_vec(way).tag)
        )
        // printf("r_idx: %x, r_tag: %x, entry-vec-addr: %x\n", r_idx, r_tag, s2_entry_vec(0).addr)
        val refill_s2_hit = refill_s2_valid && refill_s2_hit_vec.orR
        // when (refill_s2_valid) {
        //   printf("refill_s2_hit: %x on set: %d, tag: %d\n", refill_s2_hit_vec.asUInt, v_idx, v_tag)
        // }

        when (refill_s2_valid && !refill_s2_hit_vec.orR) {
          val v_valid_vec = valid.map(_(v_idx)).asUInt
          val v_valid_vec_q = Reg(UInt(boomParams.nL2HTLBWays.W))
        
          // replacement way
          val v_l2_plru_way = Reg(UInt(log2Ceil(boomParams.nL2HTLBWays max 1).W))
          v_valid_vec_q := v_valid_vec
          v_l2_plru_way := (if (boomParams.nL2HTLBWays > 1) l2_plru.way(v_idx)
                            else 0.U)

          val entry = Wire(new L2HTLBEntry(boomParams.nL2HTLBSets))
          entry.try_phys := victim_hte.try_phys
          entry.addr := victim_hte.addr
          entry.tag := v_tag
          entry.phys := victim_hte.phys
          entry.ae := victim_hte.ae
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
          midas.targetutils.SynthesizePrintf(printf("[Hn%x,%d,%x,%d\n", entry.addr, v_idx, wmask, v_tag))
          // midas.targetutils.SynthesizePrintf(printf("[HTW] Inserting with addr: %x into set %d, way (%x) %d (tag)\n", entry.addr, v_idx, wmask, v_tag))

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

      val s1_rdata = ram.read(r_idx, s0_valid)
      val s2_rdata =
        s1_rdata.map(s1_rdway => code.decode(RegEnable(s1_rdway, s1_valid)))

      val s2_valid_vec = RegEnable(r_valid_vec, s1_valid)
      val s2_error = (0 until boomParams.nL2HTLBWays)
        .map(way => s2_valid_vec(way) && s2_rdata(way).error)
        .orR

      when(s2_valid && s2_error) { valid.foreach { _ := 0.U } }

      val s2_entry_vec =
        s2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(boomParams.nL2HTLBSets)))
      val s2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
        s2_valid_vec(way) && (r_tag === s2_entry_vec(way).tag)
      )

      val s2_hit = s2_valid && s2_hit_vec.orR

      io.dpath.perf.l2miss := s2_valid && !(s2_hit_vec.orR)
      when(io.dpath.perf.l2miss) {
        midas.targetutils.SynthesizePrintf(printf("[H2m\n"))
      }
      midas.targetutils.PerfCounter(io.dpath.perf.l2miss, "l2_htlb_miss", "L2 HTLB Miss")

      when(s2_hit) {
        assert((PopCount(s2_hit_vec) === 1.U) || s2_error, "L2 HTLB multi-hit")
      }

      val s2_hte = Wire(new HTE)
      val s2_hit_entry = Mux1H(s2_hit_vec, s2_entry_vec)
      s2_hte.addr := s2_hit_entry.addr
      s2_hte.reserved := 0.U
      s2_hte.try_phys := s2_hit_entry.try_phys
      s2_hte.phys := s2_hit_entry.phys
      s2_hte.ae := s2_hit_entry.ae

      for (way <- 0 until boomParams.nL2HTLBWays) {
        ccover(
          s2_hit && s2_hit_vec(way),
          s"L2_HTLB_HIT_WAY$way",
          s"L2 HTLB hit way$way"
        )
      }

      when(s2_hit) {
        printf("[HTW] Hit with addr: %x (%d)\n", s2_hit_entry.addr, s2_hit_entry.tag)
        when (s2_hit_entry.phys) {
          // midas.targetutils.SynthesizePrintf(printf("Hit on physical, invalidating with Mask (of set): %x onto hit vec %x\n", r_idx, s2_hit_vec.asUInt))
          val mask = UIntToOH(r_idx)
          for (way <- 0 until boomParams.nL2HTLBWays) {
            // midas.targetutils.SynthesizePrintf(printf("Way: %d, Valid: %x, Mask: %x\n", way.U, valid(way), mask))
            when(s2_hit_vec(way)) {
              valid(way) := valid(way) & ~mask
            }
          }
        }
      }

      io.dpath.htDumped := set_idx === boomParams.nL2HTLBSets.U && state === s_dump
      io.dpath.htInvald := state === s_invalidated

      when (io.dpath.htDumped) {
        midas.targetutils.SynthesizePrintf(printf("[H2D\n"))
      }

      val pipeline_stage = RegInit(0.U(2.W))

      val dr_valid_vec = valid.map(_(set_idx(idxBits-1,0))).asUInt
      val ds0_valid = state === s_dump && pipeline_stage === 0.U
      val ds1_valid = RegNext(ds0_valid)
      val ds2_valid = RegNext(ds1_valid) && pipeline_stage === 2.U

      val ds1_rdata = ram.read(set_idx(idxBits-1,0), ds0_valid)
      val ds2_rdata =
        ds1_rdata.map(ds1_rdway => code.decode(RegEnable(ds1_rdway, ds1_valid)))

      val ds2_error = (0 until boomParams.nL2HTLBWays)
        .map(way => dr_valid_vec(way) && ds2_rdata(way).error)
        .orR
      when(ds2_valid && ds2_error) { valid.foreach { _ := 0.U } }

      val ds2_entry_vec =
        ds2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(boomParams.nL2HTLBSets)))

      val ds2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
        dr_valid_vec(way)
      )

      val ds2_hit = ds2_valid && ds2_hit_vec.orR && !ds2_error
      val ds2_hte = Wire(new L2HTLBEntry(boomParams.nL2HTLBSets))

      val way_idx_update = ((state === s_dump && !ds2_hit && pipeline_stage === 3.U) || (state === s_dump_wait && mem_resp_valid))
      val set_idx_update = (way_idx === (boomParams.nL2HTLBWays- 1).U) && (way_idx_update)

      val way_clear = (set_idx_update || (state === s_dump_req && way_idx === boomParams.nL2HTLBWays.U)) || (state === s_dump && next_state === s_ready)
      val set_clear = (state === s_dump && next_state === s_ready) 

      val is_dumping = state === s_dump && way_idx < boomParams.nL2HTLBWays.U && set_idx < boomParams.nL2HTLBSets.U
      val way = RegInit(0.U(log2Ceil(boomParams.nL2HTLBWays).W))
      when ((way_idx_update && !set_idx_update) || RegNext(way_idx_update && set_idx_update)) {
        val new_way = l2_plru.way(set_idx)
        way := l2_plru.way(set_idx)
      }

      when (is_dumping) {
        val ds2_hit_entry = Mux1H(UIntToOH(way), ds2_entry_vec)
        ds2_hte.addr := ds2_hit_entry.addr
        ds2_hte.tag := ds2_hit_entry.tag
        ds2_hte.try_phys := DontCare
        ds2_hte.phys := DontCare
        ds2_hte.ae := DontCare

        when(ds2_hit && dr_valid_vec(way)) {
          hid_to_dump := Cat(Mux1H(UIntToOH(way), ds2_entry_vec).tag, set_idx(idxBits-1,0))
          // midas.targetutils.SynthesizePrintf(printf(
          //   "[HTW]  L2Entry: %d: %d, Valid(%d) %x - %x\n",
          //   set_idx(idxBits-1,0), 
          //   way,
          //   dr_valid_vec(way),
          //   Cat(Mux1H(UIntToOH(way), ds2_entry_vec).tag, set_idx(idxBits-1,0)),
          //   ds2_hte.addr
          // ))
          next_state := s_dump_req
          l2_plru.access(set_idx(idxBits-1,0), way)
        } .elsewhen(ds2_valid) {
          l2_plru.access(set_idx(idxBits-1,0), way)
        }
      }.otherwise {
        ds2_hte := DontCare
      }

      way_idx := Mux(way_clear, 0.U, Mux(way_idx_update, way_idx + 1.U, way_idx))

      // Update pipeline stage
      when ((pipeline_stage >= 2.U && way_idx_update) || (set_clear && way_clear)) {
        pipeline_stage := 0.U
      } .elsewhen(state === s_dump ) {
        // pipeline_stage := Mux(pipeline_stage === 2.U, 2.U, pipeline_stage + 1.U)
        pipeline_stage := pipeline_stage + 1.U
      }

      set_idx := Mux(set_idx_update, 
                     set_idx + 1.U, 
                     Mux(state === s_dump_req && way_idx === boomParams.nL2HTLBWays.U, 
                         hid_to_dump(idxBits-1,0) + 1.U, 
                         Mux(set_clear, 0.U,
                         set_idx)))

      // printf("SetIdx: %d, WayIdx: %d\n", set_idx, way_idx)
      // printf("SetIdxUpdate: %d, WayIdxUpdate: %d\n", set_idx_update, way_idx_update)
      // printf("Pipeline Stage: %d\n", pipeline_stage)
      // printf("WayClear: %d, SetClear: %d\n", way_clear, set_clear)

      val htInval = io.dpath.customCSRs.htInval
      when (htInval.orR) {
        when (htInval(handleBits -1, 0) === ((BigInt(1) << handleBits) - 1).U) {
          // midas.targetutils.SynthesizePrintf(printf("[HI\n"))
          for (way <- 0 until boomParams.nL2HTLBWays) {
            valid(way) := 0.U
          }
          next_state := s_invalidated
        } .otherwise {
          // midas.targetutils.SynthesizePrintf(printf("[Hi%x\n", htInval(handleBits -1, 0)))

          val (i_tag, i_idx) = Split(htInval(handleBits-1, 0), idxBits)

          def hit_vec(enable_signal: Bool, set_idx: UInt, tag: UInt) = {
            val valid_vec = valid.map(_(set_idx)).asUInt

            val s1_valid = RegNext(enable_signal)
            val s2_valid = RegNext(s1_valid)
            val s1_rdata = ram.read(set_idx, s0_valid)
            val s2_rdata =
              s1_rdata.map(s1_rdway => code.decode(RegEnable(s1_rdway, s1_valid)))
            val s2_valid_vec = RegEnable(valid_vec, s1_valid)

            val s2_error = (0 until boomParams.nL2HTLBWays)
              .map(way => s2_valid_vec(way) && s2_rdata(way).error)
              .orR

            when(s2_valid && s2_error) { valid.foreach { _ := 0.U } }

            val s2_entry_vec =
              s2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(boomParams.nL2HTLBSets)))
            val s2_hit_vec = (0 until boomParams.nL2HTLBWays).map(way =>
              s2_valid_vec(way) && (tag === s2_entry_vec(way).tag)
            )

            (s2_valid, s2_hit_vec, s2_error)
          }

          val (s2_valid, s2_hit_vec, s2_error) = hit_vec(state === s_invalidating, i_idx, i_tag)
          val s2_hit = s2_valid && s2_hit_vec.orR

          when (s2_hit) {
            val mask = UIntToOH(i_idx)
            for (way <- 0 until boomParams.nL2HTLBWays) {
              // printf("Way: %d, Valid: %x, Mask: %x\n", way.U, valid(way), mask)
              when(s2_hit_vec(way)) {
                valid(way) := valid(way) & ~mask
              }
            }
            assert((PopCount(s2_hit_vec) === 1.U) || s2_error, "L2 HTLB multi-hit")
          }

          next_state := Mux(s2_valid, s_invalidated, s_invalidating)
        }
      }
    
      when (reset.asBool || io.dpath.clear_htlb) {
        when (io.dpath.clear_htlb) {
          // midas.targetutils.SynthesizePrintf(printf("[H2C\n"))
        }
        for (way <- 0 until boomParams.nL2HTLBWays) {
          valid(way) := 0.U
        }
      }

      (s2_hit, s2_error, s2_hte, Some(ram))
    }

    // Debug HTW response
    val tmp = mem_resp_data.asTypeOf(new HTE())
    val pte = WireDefault(tmp)
    pte.try_phys := true.B
    pte.phys := false.B
    pte.ae := RegNext(io.mem.s2_xcpt.asUInt =/= 0.U)

    found_hte := OptimizationBarrier(
      Mux(l2_hit && !l2_error, l2_hte, Mux(mem_resp_valid, pte, found_hte))
    )

    val l1_htlb_size = boomParams.nL1HTLBSets*boomParams.nL1HTLBWays
    val hte_dst_addr = io.dpath.customCSRs.htDump + (l1_htlb_size*4).U + dumped_entry_idx*4.U

    // Finite State Machine Logic
    switch(state) {
      is(s_ready) {
        next_state := Mux(
          io.requestor.req.valid, s_req,
          Mux(io.requestor.evict.valid, s_victim1, 
          Mux(io.requestor.l1_dumped && io.dpath.customCSRs.htDump.orR, s_dump,
          Mux(io.dpath.customCSRs.htInval.orR, s_invalidating, 
          s_ready)
        )))

        victim_hte := Mux(io.requestor.evict.valid, io.requestor.evict.bits, victim_hte)
      }
      is(s_req) {
        if (boomParams.enableTwoStageHTW && false) {
          // val (cache, _) = top_level_cache.get
          // val cache_hit = cache(getIndex(hid)).valid && 
          //                cache(getIndex(hid)).tag === getTag(hid)
          
          // midas.targetutils.SynthesizePrintf(printf(
          //   "[H2W1%d,%x,%x,%x\n",
          //   cache_hit,
          //   getIndex(hid),
          //   getTag(hid),
          //   io.mem.req.bits.addr
          // ))
          
          // when (cache_hit) {
          //   next_state := s_req2
          // }.otherwise {
          //   next_state := Mux(io.mem.req.fire, s_wait1, s_req)
          // }
        } else {
          next_state := Mux(io.mem.req.fire, s_wait1, s_req)
        }
      }
      is(s_wait1) {
        next_state := Mux(l2_hit, s_req, s_wait2)
      }
      is (s_wait2) {
        next_state := Mux(io.mem.s2_nack, s_req, Mux(io.mem.s2_xcpt.asUInt =/= 0.U, s_req, s_wait3))
      }
      is (s_wait3) {
        if (boomParams.enableTwoStageHTW) {
          next_state := Mux(mem_resp_valid, s_req2, Mux(io.mem.s2_nack, s_req, s_wait3))
          when (mem_resp_valid) {
            // midas.targetutils.SynthesizePrintf(printf(
            //   "[H2W2%x,%x\n",
            //   mem_resp_data.asTypeOf(UInt(64.W)).asUInt,
            //   hid(entries_per_ht_bits-1, 0),
            // ))
          }
          inner_walk_base := mem_resp_data.asTypeOf(UInt(64.W)).asUInt
        } else {
          next_state := Mux(mem_resp_valid, s_ready, Mux(io.mem.s2_nack, s_req, s_wait3))
          resp_valid := mem_resp_valid
          when (mem_resp_valid) {
            // midas.targetutils.SynthesizePrintf(printf(
            //   "[H2n%x,%x,%d,%d\n",
            //   pte.reserved,
            //   pte.addr,
            //   pte.try_phys,
            //   pte.ae
            // ))
          }
        }
      }
      is (s_req2) {
        next_state := Mux(io.mem.req.fire, s_wait4, s_req2)
        // midas.targetutils.SynthesizePrintf(printf(
        //   "[H2W3%x,%x,%x\n",
        //   io.mem.req.bits.addr,
        //   inner_walk_base,
        //   hid(entries_per_ht_bits-1, 0)
        // ))
      }
      is (s_wait4) {
        next_state := s_wait5 
      }
      is (s_wait5) {
        next_state := Mux(io.mem.s2_nack, s_req2, Mux(io.mem.s2_xcpt.asUInt =/= 0.U, s_req2, s_wait6))
      }
      is (s_wait6) {
        next_state := Mux(mem_resp_valid, s_ready, Mux(io.mem.s2_nack, s_req2, s_wait6))
        resp_valid := mem_resp_valid
        when (mem_resp_valid) {
          midas.targetutils.SynthesizePrintf(printf(
            "[H2n%x,%x,%d,%d\n",
            pte.reserved,
            pte.addr,
            pte.try_phys,
            pte.ae
          ))
        }
      }
      is(s_victim1) {
        next_state := s_victim2
        midas.targetutils.SynthesizePrintf(printf("[H2n%d,%x,%d,%d,%d\n", victim_hte.hid, victim_hte.addr, victim_hte.phys, victim_hte.try_phys, victim_hte.ae))
      }
      is(s_victim2) {
        next_state := s_victim3
      }
      is(s_victim3) {
        next_state := s_ready
        midas.targetutils.SynthesizePrintf(printf("[H2v%d,%x,%d,%d,%d\n", victim_hte.hid, victim_hte.addr, victim_hte.phys, victim_hte.try_phys, victim_hte.ae))
      }
      is(s_invalidated) {
        // midas.targetutils.SynthesizePrintf(printf("[HiI\n"))
        next_state := s_ready
      }
      is (s_dump) {
        when (!io.dpath.customCSRs.htDump.orR) {
          dumped_entry_idx := 0.U
          next_state := s_ready
          printf("[HTW] Finished dumping all entries\n")
        }
      }
      is (s_dump_req) {
        // midas.targetutils.SynthesizePrintf(printf("[H2d%d,%x\n", hid_to_dump, hte_dst_addr))
        next_state := Mux(io.mem.req.fire, s_dump_wait, s_dump_req)
      }
      is (s_dump_wait) {
        // midas.targetutils.SynthesizePrintf(printf("[H2Dd%x,%x,%d\n", hid_to_dump, hte_dst_addr, mem_resp_valid))
        next_state := Mux(mem_resp_valid, s_dump, Mux(io.mem.s2_nack, s_dump_req, s_dump_wait))

        when (next_state === s_dump_replay_pending) {
          // midas.targetutils.SynthesizePrintf(printf("[H2N%d,%d\n", dumped_entry_idx, hid_to_dump))
        }. elsewhen(next_state === s_dump) {
          // midas.targetutils.SynthesizePrintf(printf("[H2D%d,%d\n", dumped_entry_idx, hid_to_dump))
          dumped_entry_idx := dumped_entry_idx + 1.U
        }
      }
      is (s_htw_replay_pending) {
        next_state := Mux(io.ptw_done, s_req, s_htw_replay_pending)
      }
      is (s_dump_replay_pending) {
        next_state := Mux(io.ptw_done, s_dump_req, s_dump_replay_pending)
      }
    }
        
    when (resp_valid) {
      midas.targetutils.SynthesizePrintf(printf(
        "[H2h%d,%x\n",
        hid,
        io.requestor.resp.bits.hte.addr
      ))
      // midas.targetutils.SynthesizePrintf(printf(
      //   "[HTW] -> [HTLB] Found HID %d to be %x\n",
      //   hid,
      //   io.requestor.resp.bits.hte.addr
      // ))

    }
    
    // Hit Handling Logic or HTW response logic
    when(l2_hit && !l2_error) {
      assert(state === s_req || state === s_wait1)
      next_state := s_ready
      resp_valid := true.B
    }

    // HT Lookup
    val walk_addr = if (boomParams.enableTwoStageHTW) {
      // val (cache, _) = top_level_cache.get
      // val cache_entry = cache(getIndex(hid))
      // val cache_hit = cache_entry.valid && cache_entry.tag === getTag(hid)
      
      Mux(state === s_req,
          htBase + (hid >> entries_per_ht_bits) * 8.U,  // First level walk
          inner_walk_base + (hid(entries_per_ht_bits-1, 0)) * 8.U)  // Second level walk using bottom bits of hid
    } else {
      htBase + hid * ((new HTE().getWidth.U) / 8.U((log2Ceil(new HTE().getWidth) + 1).W))
    }

    // Prepare Memory Request
    io.mem.keep_clock_enabled := false.B

    io.mem.req.valid := state === s_req  || state === s_dump_req || state === s_req2
    io.mem.req.bits.phys := io.requestor.pht_enabled && !(state === s_dump_req)
    io.mem.req.bits.cmd := Mux(state === s_dump_req, M_XWR, M_XRD)
    io.mem.req.bits.size := Mux(state === s_dump_req, 2.U, log2Ceil(xLen / 8).U)
    io.mem.req.bits.signed := false.B
    io.mem.req.bits.addr := Mux(state === s_dump_req, hte_dst_addr, walk_addr)
    io.mem.req.bits.idx.foreach(_ := Mux(state === s_dump_req, hte_dst_addr, walk_addr))
    io.mem.req.bits.dprv := Mux(io.requestor.pht_enabled && !(state === s_dump_req), PRV.S.U, PRV.U.U) // HTW accesses are U-mode by definition
    io.mem.req.bits.dv := false.B
    io.mem.req.bits.tag := DontCare
    io.mem.req.bits.no_resp := false.B
    io.mem.req.bits.no_alloc := DontCare
    io.mem.req.bits.no_xcpt := DontCare
    io.mem.req.bits.data := DontCare
    io.mem.req.bits.mask := DontCare

    // TODO: This may need to change if we get an exception in the middle of a handle table walk
    io.mem.s1_kill := l2_hit || (state =/= s_wait1 && state =/= s_dump_wait && state =/= s_wait4)
    io.mem.s1_data.data := Mux(state === s_dump_wait, hid_to_dump, 0.U)
    io.mem.s1_data.mask := Mux(state === s_dump_wait, ((1 << coreDataBytes) - 1).U, 0.U)
    io.mem.s2_kill := false.B

    // when (io.mem.s2_xcpt.asUInt =/= 0.U) {
    //   midas.targetutils.SynthesizePrintf(printf("[H2E%x\n", io.mem.s2_xcpt.asUInt))
    // }

    // // Add near memory request logic:
    // when (io.mem.req.valid) {
    //   midas.targetutils.SynthesizePrintf(printf(
    //     "[H2R%d,%x,%d,%d\n",
    //     state,
    //     walk_addr,
    //     io.mem.req.bits.phys,
    //     io.mem.s1_kill
    //   ))
    // }

    // Add near s1_kill logic:
    // when (io.mem.s1_kill && RegNext(io.mem.req.valid)) {
    //   midas.targetutils.SynthesizePrintf(printf(
    //     "[H2K%d,%d\n",
    //     l2_hit,
    //     state
    //   ))
    // }

    // /*
    //  HTW total latency counter
    // */
    // val walk_active = RegInit(false.B)
  
    // when (state === s_req && next_state === s_wait1) {
    //   walk_active := true.B
    // } .elsewhen (walk_active && resp_valid) {
    //   walk_active := false.B
    // }

    // midas.targetutils.PerfCounter(
    //   resp_valid && walk_active, 
    //   "htw_total_latency", 
    //   "Handle Table Walk Total Latency (cycles for both stages if two-stage walk)"
    // )
  }

  private def ccover(cond: Bool, label: String, desc: String)(implicit
      sourceInfo: SourceInfo
  ) =
    if (usingVM) property.cover(cond, s"HTW_$label", "MemorySystem;;" + desc)
}
