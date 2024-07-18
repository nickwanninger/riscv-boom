package boom.v4.lsu

import chisel3._
import chisel3.util._
import chisel3.experimental.SourceInfo

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.constants._
import freechips.rocketchip.rocket.HellaCacheIO

import boom.v4.common._
import freechips.rocketchip.rocket.PRV.U
import freechips.rocketchip.tilelink.TLMessages.c

class HTE(implicit p: Parameters) extends BoomBundle()(p) {
  val small = Bool()
  val frozen = Bool()
  val reserved = UInt((64 - maxSVAddrBits - 2).W)
  val addr = UInt(maxSVAddrBits.W)
}

class L2HTLBEntry(nSets: Int)(implicit p: Parameters) extends BoomBundle()(p) {
  val idxBits = log2Ceil(nSets)
  val tagBits = handleBits - idxBits
  val tag = UInt(tagBits.W)
  val frozen = Bool()
  val small = Bool()
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
  val small = Bool()
  val phys = Bool()
  val hid = UInt(handleBits.W)
}

class HTLBHTWIO(implicit p: Parameters) extends BoomBundle()(p) {
  val req = Decoupled(Valid(new HTWReq))
  val resp = Flipped(Valid(new HTWResp))
  val evict = Decoupled(new EvictionReq)
  val evict_resp = Input(Bool())

  val l1miss = Bool()
  val l1hit = Bool()
}

class HTWPerfEvents(implicit p: Parameters) extends BoomBundle()(p) {
  val l2miss = Bool()
  val l2hit = Bool()
  val l1miss = Bool()
  val l1hit = Bool()
}

class DatapathHTWIO(implicit p: Parameters) extends BoomBundle()(p) {
  val sfence = Flipped(Valid(new SFenceReq))
  val perf = Output(new HTWPerfEvents())
  val customCSRs = Flipped(coreParams.customCSRs)
  val htDumped = Output(Bool())
  val clock_enabled = Output(Bool())
}

class HTW(implicit p: Parameters) extends BoomModule()(p) {
  require(maxSVAddrBits == 39)
  require(new HTE().getWidth == 64)
  val io = IO(new Bundle {
    val requestor = Flipped(new HTLBHTWIO)
    val mem = new HellaCacheIO
    val dpath = new DatapathHTWIO
  })

  // State Machine
  val s_ready :: s_req :: s_wait :: s_victim :: s_dumping :: s_dumping_wait :: Nil = Enum(6)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)
  val l2_refill_wire = Wire(Bool())
  state := OptimizationBarrier(next_state)

  val resp_valid = RegNext(false.B)

  io.dpath.customCSRs := DontCare

  val clock_en =
    state =/= s_ready || l2_refill_wire || io.requestor.req.valid || io.dpath.sfence.valid || io.dpath.customCSRs.disableDCacheClockGate
  io.dpath.clock_enabled := usingVM.B && clock_en
  val gated_clock =
    if (!usingVM || !tileParams.dcache.get.clockGate) clock
    else ClockGate(clock, clock_en, "ptw_clock_gate")
  withClock(gated_clock) {
    val l2_refill = RegNext(false.B)
    l2_refill_wire := l2_refill

    io.requestor.req.ready := (state === s_ready) && !l2_refill_wire

    val invalidated = Reg(Bool())
    val r_req = Reg(new HTWReq)
    val r_hte = Reg(new HTE)
    val v_hte = Reg(new HTE)
    val v_hid = Reg(UInt(handleBits.W))

    invalidated := io.dpath.sfence.valid || (invalidated && state =/= s_ready)

    /* debug print for handle table walks */
    when(io.mem.req.valid && state === s_wait) {
      printf(
        "[HTW] -> [Mem] Looking up HID: %d at %x\n",
        r_req.hid,
        io.mem.req.bits.addr
      )
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
    // TODO: what if the HTE is invalid?
    io.requestor.resp.valid := resp_valid
    io.requestor.resp.bits.hte := r_hte

    /* debug print for response from HTW */
    when(io.requestor.resp.valid) {
      printf(
        "[HTW] -> [HTLB] Found HID %d to be %x\n",
        io.requestor.req.bits.bits.hid,
        io.requestor.resp.bits.hte.addr
      )
    }

    io.requestor.evict.ready := state === s_ready

    // Finite State Machine Logic
    switch(state) {
      is(s_ready) {
        next_state := Mux(
          io.requestor.req.valid,
          s_req,
          Mux(io.requestor.evict.valid, s_victim, s_ready)
        )

        when(io.requestor.req.valid) {
          r_req := io.requestor.req.bits.bits
        }

        when(io.requestor.evict.valid) {
          v_hte.addr := io.requestor.evict.bits.addr
          v_hte.frozen := false.B
          v_hte.reserved := 0.U
          v_hte.small := io.requestor.evict.bits.small
          v_hid := io.requestor.evict.bits.hid
        }
      }
      is(s_victim) {
        next_state := Mux(
          io.dpath.sfence.valid,
          s_ready,
          Mux(invalidated, s_ready, s_victim)
        )
      }
    }

    // Debug HTW response
    val tmp = mem_resp_data.asTypeOf(new HTE())
    val pte = WireDefault(tmp)
    when(mem_resp_valid && state === s_wait) {
      printf(
        "[HTW] Found HTE - Frozen: %x, Reserved: %x, Addr: %x, Small: %d\n",
        pte.frozen,
        pte.reserved,
        pte.addr,
        pte.small
      )
    }

    // Victim Cache Logic
    when(state === s_victim) {
      printf("Victim Entry: %d - %x\n", v_hid, v_hte.addr)
    }

    l2_refill := state === s_victim
    io.requestor.evict.ready := state === s_ready

    val nL2TLBSets = coreParams.nL2TLBEntries / coreParams.nL2TLBWays
    val idxBits = log2Ceil(nL2TLBSets)
    val d_ways = RegInit(0.U(log2Ceil(coreParams.nL2TLBWays).W))
    val d_set = RegInit(0.U(idxBits.W))
    val dumped_htlb_idx = RegInit(0.U(idxBits.W))

    val (l2_hit, l2_error, l2_hte, l2_htlb_ram, d_hte) = {
      val code = new ParityCode
      require(isPow2(coreParams.nL2TLBEntries))
      require(isPow2(coreParams.nL2TLBWays))
      require(coreParams.nL2TLBEntries >= coreParams.nL2TLBWays)
      require(isPow2(nL2TLBSets))

      val l2_plru = new SetAssocLRU(nL2TLBSets, coreParams.nL2TLBWays, "plru")

      val ram = DescribedSRAM(
        name = "l2_htlb_ram",
        desc = "L2 HTLB",
        size = nL2TLBSets,
        data = Vec(
          coreParams.nL2TLBWays,
          UInt(code.width(new L2HTLBEntry(nL2TLBSets).getWidth).W)
        )
      )

      // val g = Reg(Vec(coreParams.nL2TLBWays, UInt(nL2TLBSets.W)))
      val valid = RegInit(
        VecInit(Seq.fill(coreParams.nL2TLBWays)(0.U(nL2TLBSets.W)))
      )
      // use r_req to construct tag
      val (r_tag, r_idx) = Split(r_req.hid, idxBits)

      /** the valid vec for the selected set(including n ways) */
      val r_valid_vec = valid.map(_(r_idx)).asUInt
      val r_valid_vec_q = Reg(UInt(coreParams.nL2TLBWays.W))
      r_valid_vec_q := r_valid_vec
      // refill with r_pte(leaf pte)
      when(l2_refill && !invalidated) {
        val (v_tag, v_idx) = Split(v_hid, idxBits)

        val v_valid_vec = valid.map(_(v_idx)).asUInt
        val v_valid_vec_q = Reg(UInt(coreParams.nL2TLBWays.W))
        
        // replacement way
        val v_l2_plru_way = Reg(UInt(log2Ceil(coreParams.nL2TLBWays max 1).W))
        v_valid_vec_q := v_valid_vec
        // replacement way
        v_l2_plru_way := (if (coreParams.nL2TLBWays > 1) l2_plru.way(v_idx)
                          else 0.U)

        val entry = Wire(new L2HTLBEntry(nL2TLBSets))
        entry.small := v_hte.small
        entry.frozen := v_hte.frozen
        entry.addr := v_hte.addr
        entry.tag := v_tag
        // if all the way are valid, use plru to select one way to be replaced,
        // otherwise use PriorityEncoderOH to select one
        val wmask =
          if (coreParams.nL2TLBWays > 1)
            Mux(
              v_valid_vec_q.andR,
              UIntToOH(v_l2_plru_way, coreParams.nL2TLBWays),
              PriorityEncoderOH(~v_valid_vec_q)
            )
          else 1.U(1.W)
        ram.write(
          v_idx,
          VecInit(Seq.fill(coreParams.nL2TLBWays)(code.encode(entry.asUInt))),
          wmask.asBools
        )
        printf("[HTW] Inserting with addr: %x into set %d, way %d (tag)\n", entry.addr, v_idx, v_tag)

        val mask = UIntToOH(v_idx)
        // printf("Mask: %x\n", mask)
        for (way <- 0 until coreParams.nL2TLBWays) {
          when(wmask(way)) {
            valid(way) := valid(way) | mask
            //   g(way) := Mux(r_pte.g, g(way) | mask, g(way) & ~mask)
          }
        }

        next_state := s_ready
      }
      when(io.dpath.sfence.valid) {
        printf("[HTW] Invalidating all entries\n")
        for (way <- 0 until coreParams.nL2TLBWays) {
          valid(way) := 0.U
        }
      }

      val s0_valid = !l2_refill
      val s1_valid = RegNext(s0_valid && io.requestor.req.valid)
      val s2_valid = RegNext(s1_valid)
      // read from tlb idx
      val s1_rdata = ram.read(r_idx, s0_valid)
      val s2_rdata =
        s1_rdata.map(s1_rdway => code.decode(RegEnable(s1_rdway, s1_valid)))
      val s2_valid_vec = RegEnable(r_valid_vec, s1_valid)
      // val s2_g_vec = RegEnable(VecInit(g.map(_(r_idx))), s1_valid)
      val s2_error = (0 until coreParams.nL2TLBWays)
        .map(way => s2_valid_vec(way) && s2_rdata(way).error)
        .orR
      when(s2_valid && s2_error) { valid.foreach { _ := 0.U } }
      // decode
      val s2_entry_vec =
        s2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(nL2TLBSets)))
      val s2_hit_vec = (0 until coreParams.nL2TLBWays).map(way =>
        s2_valid_vec(way) && (r_tag === s2_entry_vec(way).tag)
      )
      // printf("r_idx: %x, r_tag: %x, entry-vec-tag: %x\n", r_idx, r_tag, s2_entry_vec(0).tag)
      val s2_hit = s2_valid && s2_hit_vec.orR
      io.dpath.perf.l2miss := s2_valid && !(s2_hit_vec.orR)
      io.dpath.perf.l2hit := s2_hit
      when(s2_hit) {
        // l2_plru.access(r_idx, OHToUInt(s2_hit_vec))
        val invl_mask = UIntToOH(r_idx)
        // printf("Invl Mask: %x\n", invl_mask)
        for (way <- 0 until coreParams.nL2TLBWays) {
            valid(way) := valid(way) & ~invl_mask
            //   g(way) := Mux(r_pte.g, g(way) | mask, g(way) & ~mask)
        }
        assert((PopCount(s2_hit_vec) === 1.U) || s2_error, "L2 HTLB multi-hit")
      }

      // for (i <- 0 until (nL2TLBSets - 1)) {
      //   val testing_valid_vec = valid(0).asUInt
      //   when (testing_valid_vec =/= 0.U) {
      //     printf("Valid: %x\n", testing_valid_vec)
      //   }
      // }

      val s2_hte = Wire(new HTE)
      val s2_hit_entry = Mux1H(s2_hit_vec, s2_entry_vec)
      s2_hte.addr := s2_hit_entry.addr
      s2_hte.frozen := s2_hit_entry.frozen
      s2_hte.reserved := 0.U
      s2_hte.small := s2_hit_entry.small

      for (way <- 0 until coreParams.nL2TLBWays) {
        ccover(
          s2_hit && s2_hit_vec(way),
          s"L2_HTLB_HIT_WAY$way",
          s"L2 HTLB hit way$way"
        )
      }

      when(s2_hit) {
        printf("[HTW] Hit with addr: %x\n", s2_hit_entry.addr)
      }

      val set_idx = RegInit(0.U((idxBits + 1).W))
      val way_idx = RegInit(0.U((log2Ceil(coreParams.nL2TLBWays) + 1).W))
      io.dpath.htDumped := set_idx === nL2TLBSets.U && state === s_dumping

      when(io.dpath.customCSRs.htDump.orR && (state =/= s_dumping && state =/= s_dumping_wait)) {
        printf("[HTW] Starting to dump L2\n")
        next_state := s_dumping
        dumped_htlb_idx := 0.U
        set_idx := 0.U
        way_idx := 0.U
      }

      val dr_valid_vec = ShiftRegister(valid.map(_(set_idx)).asUInt, 1)
      val ds0_valid = state === s_dumping
      val ds1_valid = RegNext(ds0_valid)
      val ds2_valid = RegNext(ds1_valid)
      // read from tlb idx
      val ds1_rdata = ram.read(set_idx, ds0_valid)
      val ds2_rdata =
        ds1_rdata.map(ds1_rdway => code.decode(RegEnable(ds1_rdway, ds1_valid)))
      val ds2_error = (0 until coreParams.nL2TLBWays)
        .map(way => dr_valid_vec(way) && ds2_rdata(way).error)
        .orR
      when(ds2_valid && ds2_error) { valid.foreach { _ := 0.U } }
      // decode
      val ds2_entry_vec =
        ds2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(nL2TLBSets)))

      val ds2_hit_vec = (0 until coreParams.nL2TLBWays).map(way =>
        dr_valid_vec(way)
      )

      val ds2_hit = ds2_valid && ds2_hit_vec.orR

      val ds2_hte = Wire(new L2HTLBEntry(nL2TLBSets))
      when (state === s_dumping && way_idx < coreParams.nL2TLBWays.U && set_idx < nL2TLBSets.U) {
        val way = l2_plru.way(set_idx)

        val ds2_hit_entry = Mux1H(UIntToOH(way), ds2_entry_vec)
        ds2_hte.addr := ds2_hit_entry.addr
        ds2_hte.tag := ds2_hit_entry.tag
        ds2_hte.frozen := DontCare
        ds2_hte.small := DontCare

        when(ds2_hit) {
          d_set := set_idx - 1.U
          printf(
            "[HTW]  L2Entry: %d: %d, Valid(%d) %x - %x\n",
            set_idx - 1.U,
            way,
            dr_valid_vec(way),
            Cat(ds2_hte.tag, set_idx - 1.U),
            ds2_hte.addr
          )
          next_state := s_dumping_wait
        }

        printf("Set: %d, Way: %d - Valid(%d)\n", set_idx, way, dr_valid_vec(way))

        l2_plru.access(set_idx, way)
      }.otherwise {
        ds2_hte := DontCare
      }

      // FIXME: this doesn't work for ways > 1
      way_idx := Mux((state === s_dumping && !ds2_hit) || (state === s_dumping_wait && mem_resp_valid), way_idx + 1.U, way_idx)

      when (way_idx === coreParams.nL2TLBWays.U && ((state === s_dumping && !ds2_hit) || (state === s_dumping_wait && mem_resp_valid))) {
        way_idx := 0.U
      }

      set_idx := Mux(way_idx === coreParams.nL2TLBWays.U && ((state === s_dumping && !ds2_hit)), set_idx + 1.U, Mux(state === s_dumping_wait && mem_resp_valid, d_set + 1.U, set_idx))

      // TODO: double check this exit condition?
      when (set_idx === nL2TLBSets.U && state === s_dumping) {
        next_state := s_ready
      }

      (s2_hit, s2_error, s2_hte, Some(ram), ds2_hte)
    }
    // printf("%d, %d, %d\n", l2_hit, l2_error, mem_resp_valid)

    r_hte := OptimizationBarrier(
      Mux(l2_hit && !l2_error, l2_hte, Mux(mem_resp_valid, pte, r_hte))
    )

    // Hit Handling Logic or HTW response logic
    when((l2_hit && !l2_error && state === s_req) || (mem_resp_valid && state =/= s_dumping_wait)) {
      next_state := s_ready
      resp_valid := true.B
    }

    // Miss Handling Logic
    when(!l2_hit && !l2_error && state === s_req) {
      next_state := s_wait
    }

    io.requestor.evict_resp := l2_refill && !invalidated

    io.dpath.perf.l2hit := l2_hit && !l2_error
    io.dpath.perf.l2miss := !l2_hit && !l2_error && mem_resp_valid
    io.dpath.perf.l1hit := io.requestor.l1hit
    io.dpath.perf.l1miss := io.requestor.l1hit

    // HT Lookup
    val hte_vaddr =
      io.dpath.customCSRs.htBase + r_req.hid * ((new HTE().getWidth.U) / 8.U)

    val d_hte_vaddr = io.dpath.customCSRs.htDump + (dcacheParams.nTLBSets*dcacheParams.nTLBWays*8).U + dumped_htlb_idx*8.U

    when (state === s_dumping_wait) {
      printf("[HTW] Dumping hid %x to %x\n", Cat(d_hte.tag, d_set), d_hte_vaddr)
    }

    when (state === s_dumping_wait && mem_resp_valid) {
      printf("[HTW] Finished dumping hid %x\n", Cat(d_hte.tag, d_set))
      dumped_htlb_idx := dumped_htlb_idx + 1.U
      next_state := s_dumping
    }

    // Prepare Memory Request
    io.mem.keep_clock_enabled := false.B

    io.mem.req.valid := state === s_wait || state === s_dumping_wait
    io.mem.req.bits.phys := false.B
    io.mem.req.bits.cmd := Mux(state === s_wait, M_XRD, M_XWR)
    io.mem.req.bits.size :=
    log2Ceil(
      xLen / 8
    ).U // TODO: confirm this makes sense
    io.mem.req.bits.signed := false.B
    io.mem.req.bits.addr := Mux(state === s_wait, hte_vaddr, d_hte_vaddr)
    io.mem.req.bits.idx.foreach(_ := Mux(state === s_wait, hte_vaddr, d_hte_vaddr)) // TODO: huh?
    io.mem.req.bits.dprv := PRV.S.U // HTW accesses are S-mode by definition
    io.mem.req.bits.dv := false.B
    io.mem.req.bits.tag := DontCare
    io.mem.req.bits.no_resp := false.B
    io.mem.req.bits.no_alloc := DontCare
    io.mem.req.bits.no_xcpt := DontCare
    io.mem.req.bits.data := Mux(state === s_wait, DontCare, Cat(d_hte.tag, d_set))
    io.mem.req.bits.mask := Mux(state === s_wait, DontCare, ((1 << coreDataBytes) - 1).U)


    // TODO: This may need to change if we get an exception in the middle of a handle table walk
    io.mem.s1_kill := l2_hit || (state =/= s_wait && state =/= s_dumping_wait)
    io.mem.s1_data := DontCare
    io.mem.s2_kill := false.B

    when (io.mem.s1_kill) {
      printf("acccidentally killed? - %d, %d\n", l2_hit, state)
    }
  }

  private def ccover(cond: Bool, label: String, desc: String)(implicit
      sourceInfo: SourceInfo
  ) =
    if (usingVM) property.cover(cond, s"HTW_$label", "MemorySystem;;" + desc)
}
