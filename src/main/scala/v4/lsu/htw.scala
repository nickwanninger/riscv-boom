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
    val small = Bool()
    val frozen = Bool()
    val addr = UInt(maxSVAddrBits.W)
}
 
class HTWReq(implicit p: Parameters) extends BoomBundle()(p)
{
    val hid = UInt(handleBits.W)
}

class HTWResp(implicit p: Parameters) extends BoomBundle()(p)
{
    val hte = new HTE
}

class EvictionReq(implicit p: Parameters) extends BoomBundle()(p)
{
    val addr = UInt(xLen.W)
    val small = Bool()
    val phys = Bool()
    val hid = UInt(handleBits.W)
}

class HTLBHTWIO(implicit p: Parameters) extends BoomBundle()(p) {
  val req = Decoupled(Valid(new HTWReq))
  val resp = Flipped(Valid(new HTWResp))
  val evict = Decoupled(new EvictionReq)
}

class HTWPerfEvents(implicit p: Parameters) extends BoomBundle()(p) {
    val l2miss = Bool()
    val l2hit = Bool()
}

class DatapathHTWIO(implicit p: Parameters) extends BoomBundle()(p)
{
    val htBase = Input(UInt(xLen.W))
    val sfence = Flipped(Valid(new SFenceReq))
    val perf = Output(new HTWPerfEvents())
}

class HTW(implicit p: Parameters) extends BoomModule()(p) {
    require(maxSVAddrBits == 39)
    require(new HTE().getWidth == 64)
    val io = IO(new Bundle {
        /** to n HTLB */
        val requestor = Flipped(new HTLBHTWIO)
        /** to HellaCache */
        val mem = new HellaCacheIO
        /** to Core
         * 
         * contains CSRs info and performance statistics
         */
        val dpath = new DatapathHTWIO
    })

    // State Machine
    val s_ready :: s_req1 :: s_wait1 :: Nil = Enum(3)
    val state = RegInit(s_ready)
    val next_state = WireDefault(state)
    state := OptimizationBarrier(next_state)
    val l2_refill_wire = Wire(Bool())

    val resp_valid = RegNext(false.B)
    val victim = RegInit(false.B)
    val next_victim_state = WireDefault(victim)
    victim := OptimizationBarrier(next_victim_state)

    io.requestor.req.ready := (state === s_ready) && !l2_refill_wire

    val invalidated = Reg(Bool())
    val r_req = Reg(new HTWReq)
    val r_hte = Reg(new HTE)
    val v_hte = Reg(new HTE)
    val v_hid = Reg(UInt(handleBits.W))

    invalidated := io.dpath.sfence.valid || (invalidated && state =/= s_ready)

    // HT Lookup
    val hte_vaddr = io.dpath.htBase + io.requestor.req.bits.bits.hid * ((new HTE().getWidth.U) / 8.U)

    // Prepare Memory Request
    io.mem.keep_clock_enabled := false.B
    io.mem.req.valid := state =/= s_ready
    io.mem.req.bits.phys := false.B
    io.mem.req.bits.cmd  := M_XRD
    io.mem.req.bits.size := log2Ceil(xLen/8).U // TODO: confirm this makes sense
    io.mem.req.bits.signed := false.B
    io.mem.req.bits.addr := hte_vaddr
    io.mem.req.bits.idx.foreach(_ := hte_vaddr) // TODO: huh?
    io.mem.req.bits.dprv := PRV.S.U   // HTW accesses are S-mode by definition
    io.mem.req.bits.dv := false.B
    io.mem.req.bits.tag := DontCare
    io.mem.req.bits.no_resp := false.B
    io.mem.req.bits.no_alloc := DontCare
    io.mem.req.bits.no_xcpt := DontCare
    io.mem.req.bits.data := DontCare
    io.mem.req.bits.mask := DontCare

    // TODO: This may need to change if we get an exception in the middle of a handle table walk
    io.mem.s1_kill := false.B
    io.mem.s1_data := DontCare
    io.mem.s2_kill := false.B

    /* debug print for handle table walks */
    when (io.mem.req.valid) {
        printf("[HTW] -> [Mem] Looking up HID: %d at %x\n", io.requestor.req.bits.bits.hid, io.mem.req.bits.addr)
    }

  val mem_resp_valid = RegNext(io.mem.resp.valid)
  val mem_resp_data = RegNext(io.mem.resp.bits.data)
  io.mem.uncached_resp.map { resp =>
    assert(!(resp.valid && io.mem.resp.valid))
    resp.ready := true.B
    when (resp.valid) {
      mem_resp_valid := true.B
      mem_resp_data := resp.bits.data
    }
  }

    // Send completed request to HTLB
    // TODO: what if the HTE is invalid?
    io.requestor.resp.valid := resp_valid
    io.requestor.resp.bits.hte := r_hte

    /* debug print for response from HTW */
    when (io.requestor.resp.valid) {
        printf("[HTW] -> [HTLB] Found HID %d to be %x\n", io.requestor.req.bits.bits.hid, io.requestor.resp.bits.hte.addr)
        printf("HTE: %x\n", r_hte.addr)
    }

    switch (state) {
        is (s_ready) {
            next_state := Mux(io.requestor.req.valid, s_req1,
               s_ready)
            when (io.requestor.req.valid) {
              r_req := io.requestor.req.bits.bits
            }

            next_victim_state := Mux(io.requestor.evict.valid, true.B, false.B)
            when (io.requestor.evict.valid) {
              v_hte.addr := io.requestor.evict.bits.addr
              v_hte.small := io.requestor.evict.bits.small
              v_hte.frozen := false.B
              v_hte.reserved := 0.U
              v_hid := io.requestor.evict.bits.hid
            }
        }
        is (s_req1) {
            when (io.mem.resp.valid) {
                next_state := s_wait1
            }
        }
        
    }

    val l2_refill = RegNext(false.B)
    l2_refill_wire := l2_refill

    val tmp = mem_resp_data.asTypeOf(new HTE())
    val pte = WireDefault(tmp)
    when (mem_resp_valid) {
      printf("PTE - Small: %x, Frozen: %x, Reserved: %x, Addr: %x\n", pte.small, pte.frozen, pte.reserved, pte.addr)
      // l2_refill := true.B
    }

    when (victim) {
          printf("Victim Entry: %d - %x\n", v_hid, v_hte.addr)
          l2_refill := true.B
    }

    io.requestor.evict.ready := state === s_ready

    // TODO: clock gate for all this later
    val (l2_hit, l2_error, l2_hte, l2_htlb_ram) = {
        val code = new ParityCode
        require(isPow2(coreParams.nL2TLBEntries))
        require(isPow2(coreParams.nL2TLBWays))
        require(coreParams.nL2TLBEntries >= coreParams.nL2TLBWays)
        val nL2TLBSets = coreParams.nL2TLBEntries / coreParams.nL2TLBWays
        require(isPow2(nL2TLBSets))
        val idxBits = log2Ceil(nL2TLBSets)

        val l2_plru = new SetAssocLRU(nL2TLBSets, coreParams.nL2TLBWays, "plru")

        val ram =  DescribedSRAM(
          name = "l2_htlb_ram",
          desc = "L2 HTLB",
          size = nL2TLBSets,
          data = Vec(coreParams.nL2TLBWays, UInt(code.width(new L2HTLBEntry(nL2TLBSets).getWidth).W))
        )

        // val g = Reg(Vec(coreParams.nL2TLBWays, UInt(nL2TLBSets.W)))
        val valid = RegInit(VecInit(Seq.fill(coreParams.nL2TLBWays)(0.U(nL2TLBSets.W))))
        // use r_req to construct tag
        val (r_tag, r_idx) = Split(r_req.hid, idxBits)
        /** the valid vec for the selected set(including n ways) */
        val r_valid_vec = valid.map(_(r_idx)).asUInt
        val r_valid_vec_q = Reg(UInt(coreParams.nL2TLBWays.W))
        val r_l2_plru_way = Reg(UInt(log2Ceil(coreParams.nL2TLBWays max 1).W))
        r_valid_vec_q := r_valid_vec
        // replacement way
        r_l2_plru_way := (if (coreParams.nL2TLBWays > 1) l2_plru.way(r_idx) else 0.U)
        // refill with r_pte(leaf pte)
        when (l2_refill && !invalidated) {
          val entry = Wire(new L2HTLBEntry(nL2TLBSets))
          entry.small := v_hte.small
          entry.frozen := v_hte.frozen
          entry.addr := v_hte.addr
          entry.tag := v_hid
          // if all the way are valid, use plru to select one way to be replaced,
          // otherwise use PriorityEncoderOH to select one
          val wmask = if (coreParams.nL2TLBWays > 1) Mux(r_valid_vec_q.andR, UIntToOH(r_l2_plru_way, coreParams.nL2TLBWays), PriorityEncoderOH(~r_valid_vec_q)) else 1.U(1.W)
          ram.write(r_idx, VecInit(Seq.fill(coreParams.nL2TLBWays)(code.encode(entry.asUInt))), wmask.asBools)
          printf("Entry to be written: %x\n", entry.addr)

          val mask = UIntToOH(r_idx)
          for (way <- 0 until coreParams.nL2TLBWays) {
            when (wmask(way)) {
              valid(way) := valid(way) | mask
            //   g(way) := Mux(r_pte.g, g(way) | mask, g(way) & ~mask)
            }
          }
          next_state := Mux(io.requestor.req.valid, s_wait1,
            Mux(io.requestor.evict.valid, s_ready, state))
        }
        // TODO: sfence happens
        /*
        when (io.dpath.sfence.valid) {
          val hg = usingHypervisor.B && io.dpath.sfence.bits.hg
          for (way <- 0 until coreParams.nL2TLBWays) {
            valid(way) :=
              Mux(!hg && io.dpath.sfence.bits.rs1, valid(way) & ~UIntToOH(io.dpath.sfence.bits.addr(idxBits+pgIdxBits-1, pgIdxBits)),
              Mux(!hg && io.dpath.sfence.bits.rs2, valid(way) & g(way),
              0.U))
          }
        }
        */

        val s0_valid = !l2_refill
        val s1_valid = RegNext(s0_valid && io.mem.req.valid)
        val s2_valid = RegNext(s1_valid)
        // read from tlb idx
        val s1_rdata = ram.read(r_idx, s0_valid)
        val s2_rdata = s1_rdata.map(s1_rdway => code.decode(RegEnable(s1_rdway, s1_valid)))
        val s2_valid_vec = RegEnable(r_valid_vec, s1_valid)
        // val s2_g_vec = RegEnable(VecInit(g.map(_(r_idx))), s1_valid)
        val s2_error = (0 until coreParams.nL2TLBWays).map(way => s2_valid_vec(way) && s2_rdata(way).error).orR
        when (s2_valid && s2_error) { valid.foreach { _ := 0.U }}
        // decode
        val s2_entry_vec = s2_rdata.map(_.uncorrected.asTypeOf(new L2HTLBEntry(nL2TLBSets)))
        val s2_hit_vec = (0 until coreParams.nL2TLBWays).map(way => s2_valid_vec(way) && (r_tag === s2_entry_vec(way).tag))
        // printf("r_idx: %x, r_tag: %x, entry-vec-tag: %x\n", r_idx, r_tag, s2_entry_vec(0).tag)
        val s2_hit = s2_valid && s2_hit_vec.orR
        io.dpath.perf.l2miss := s2_valid && !(s2_hit_vec.orR)
        io.dpath.perf.l2hit := s2_hit
        when (s2_hit) {
          l2_plru.access(r_idx, OHToUInt(s2_hit_vec))
          assert((PopCount(s2_hit_vec) === 1.U) || s2_error, "L2 HTLB multi-hit")
        }

        val s2_hte = Wire(new HTE)
        val s2_hit_entry = Mux1H(s2_hit_vec, s2_entry_vec)
        s2_hte.addr := s2_hit_entry.addr
        s2_hte.frozen := s2_hit_entry.frozen
        s2_hte.small := s2_hit_entry.small
        s2_hte.reserved := 0.U

        for (way <- 0 until coreParams.nL2TLBWays) {
          ccover(s2_hit && s2_hit_vec(way), s"L2_HTLB_HIT_WAY$way", s"L2 HTLB hit way$way")
        }

        when (s2_hit) {
          printf("s2_hit_entry_addr: %x\n", s2_hit_entry.addr)
        }

        (s2_hit, s2_error, s2_hte, Some(ram))
    }
    printf("%d, %d, %d\n", l2_hit, l2_error, mem_resp_valid)
    r_hte := OptimizationBarrier(Mux(l2_hit && !l2_error, l2_hte,
                                 Mux(mem_resp_valid, pte, r_hte)))

  when ((l2_hit && !l2_error && state === s_wait1) || mem_resp_valid) {
    next_state := s_ready
    resp_valid := true.B
  }

  io.dpath.perf.l2hit := false.B
  io.dpath.perf.l2miss := false.B

  private def ccover(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    if (usingVM) property.cover(cond, s"HTW_$label", "MemorySystem;;" + desc)
}