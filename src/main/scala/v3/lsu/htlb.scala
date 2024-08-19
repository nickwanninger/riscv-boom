package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import boom.v3.common._
import freechips.rocketchip.tile.CoreBundle
import freechips.rocketchip.jtag.JtagState.State.width
import freechips.rocketchip.tilelink.TLMessages.d

class HTLBReq(implicit p: Parameters) extends BoomBundle()(p) {
  val haddr = UInt(xLen.W)
  val passthrough = Bool()
}

class HTLBResp(implicit p: Parameters) extends BoomBundle()(p) {
  val addr = UInt(xLen.W)
  val miss = Bool()
  val phys = Bool()
  val small = Bool()
}

class TLBResp(implicit p: Parameters) extends BoomBundle()(p) {
  val hid = UInt(xLen.W)
  val paddr = UInt(ppnBits.W)
}

case class HTLBConfig(
    nSets: Int,
    nWays: Int
)

class HTLB(cfg: TLBConfig)(implicit p: Parameters) extends BoomModule()(p) {
  val io = IO(new Bundle {
    val req = Flipped(Vec(memWidth, Decoupled(new HTLBReq)))
    val resp = Vec(memWidth, new HTLBResp)
    val htw = new HTLBHTWIO
    val tlb = Flipped(Vec(memWidth, Valid(new TLBResp)))
    val mem = new HellaCacheIO
    val htDump = Input(UInt(xLen.W))
  })

  class HTLBEntryData() extends Bundle() {
    val phys = Bool()
    val addr = UInt(xLen.W)
    val immovable = Bool()
    val small = Bool()
  }

  class Entry() extends Bundle {
    val tag = UInt(handleBits.W)
    val data = UInt(new HTLBEntryData().getWidth.W)
    val valid = Bool()

    def getData() = data.asTypeOf(new HTLBEntryData)
    def hit(hid: UInt) = {
      valid && tag === hid
    }

    def insert(hid: UInt, entry: HTLBEntryData) = {
      this.tag := hid

      valid := true.B
      data := entry.asUInt
    }

    def invalidate() = { valid := false.B }

    def lock(hid: UInt) = {
      getData().immovable := true.B
    }

    def unlock(hid: UInt) = {
      getData().immovable := false.B
    }
  }

  // Utilities
  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))
  val hm_enabled = widthMap(w => !io.req(w).bits.passthrough)
  val hid = widthMap(w => io.req(w).bits.haddr(xLen - 2, handleBits))

  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets * cfg.nWays, new Entry()))

  // State Machine
  val s_ready :: s_request :: s_wait :: s_wait_invalidate :: s_victim_wait :: s_ht_dump :: s_ht_dump_wait :: s_ht_dumped :: Nil = Enum(8)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)
  state := OptimizationBarrier(next_state)

  // Refill State
  val do_refill = io.htw.resp.valid
  val r_refill_tag = Reg(UInt(handleBits.W))
  val r_sectored_repl_addr = Reg(UInt(log2Ceil(entries.size).W))
  val r_sectored_hit_addr = Reg(UInt(log2Ceil(entries.size).W))
  val r_sectored_hit = Reg(Bool())

  // Hit Logic
  val hitsVec =
    widthMap(w => VecInit(entries.map(hm_enabled(w) && _.hit(hid(w)))))
  val real_hits = widthMap(w => hitsVec(w).asUInt)
  val htlb_hit = widthMap(w => real_hits(w).orR)
  val htlb_miss = widthMap(w => hm_enabled(w) && !htlb_hit(w))

  val plru = new PseudoLRU(entries.size)
  for (w <- 0 until memWidth) {
    when(io.req(w).valid && hm_enabled(w)) {
      plru.access(OHToUInt(real_hits(w)))
    }
  }

  // Miss Logic
  // val victim_ready :: victim_wait :: Nil = Enum(2)
  // val victim = RegInit(victim_ready)
  val victim_entry = Reg(new Entry())
  val vic = victim_entry.data.asTypeOf(new HTLBEntryData)

  when(io.htw.evict.valid) {
    midas.targetutils.SynthesizePrintf(printf(
      "[HTLB] Victim Entry (%x): %x\n",
      victim_entry.tag,
      vic.addr
    ))
  }

  io.htw.evict.valid := state === s_victim_wait
  io.htw.evict.bits.hid := victim_entry.tag
  io.htw.evict.bits.addr := vic.addr
  io.htw.evict.bits.phys := vic.phys
  io.htw.evict.bits.small := vic.small
  io.htw.l1_dumped := state === s_ht_dumped

  // Utilities to help respond
  val addr = widthMap(w =>
    Mux1H(
      hitsVec(w) :+ !hm_enabled(w),
      entries.map(_.data.asTypeOf(new HTLBEntryData).addr)
    )
  )
  val phys = widthMap(w =>
    Mux1H(
      hitsVec(w) :+ !hm_enabled(w),
      entries.map(_.data.asTypeOf(new HTLBEntryData).phys)
    )
  )
  val small = widthMap(w =>
    Mux1H(
      hitsVec(w) :+ !hm_enabled(w),
      entries.map(_.data.asTypeOf(new HTLBEntryData).small)
    )
  )

  // Send response to LSU
  for (w <- 0 until memWidth) {
    // handle original case
    val sum = io.req(w).bits.haddr
    val ea_sign = Mux(
      sum(vaddrBits - 1),
      ~sum(63, vaddrBits) === 0.U,
      sum(63, vaddrBits) =/= 0.U
    )
    val effective_address = Cat(ea_sign, sum(vaddrBits - 1, 0)).asUInt

    io.req(w).ready := true.B
    io.resp(w).miss := do_refill || htlb_miss(w)
    io.resp(w).addr := Mux(
      io.req(w).bits.passthrough,
      effective_address,
      addr(w) + io.req(w).bits.haddr(handleBits - 1, 0)
    )
    io.resp(w).phys := phys(w) && hm_enabled(w)
    io.resp(w).small := small(w)

    when(!io.resp(w).miss && io.req(w).valid && !io.req(w).bits.passthrough) {
       midas.targetutils.SynthesizePrintf(printf(
        "[HTLB] -> [LSU] %x %d (for %x) (paddr: %x)\n",
        io.resp(w).addr,
        io.resp(w).phys,
        io.req(w).bits.haddr,
        addr(w) + io.req(w).bits.haddr(handleBits - 1, 0)
      ))
    }
  }

  for (w <- 0 until memWidth) {
    when(
      io.req(w).fire && htlb_miss(w) && state === s_ready && !io
        .req(w)
        .bits
        .passthrough
    ) {
      next_state := s_request
      r_refill_tag := hid(w)

      r_sectored_repl_addr := replacementEntry(entries, plru.way)
      r_sectored_hit_addr := OHToUInt(real_hits(w))
      r_sectored_hit := real_hits(w).orR
    }

    when(io.htw.req.valid) {
      midas.targetutils.SynthesizePrintf(printf("[HTLB] -> [HTW] Looking up hid: %x\n", io.htw.req.bits.bits.hid))
    }

    when(io.tlb(w).valid) {
      // debug print for small handlen optimization
      printf("[HTLB] paddr for hid %x: %x\n", io.tlb(w).bits.hid(xLen - 2, handleBits), io.tlb(w).bits.paddr)
      for ((e, i) <- entries.zipWithIndex) when(e.hit(io.tlb(w).bits.hid(xLen - 2, handleBits))) {
        // e.setPAddr(io.tlb(w).bits.paddr)

        val ppn = io.tlb(w).bits.paddr
        val new_entry = Wire(new HTLBEntryData())
        new_entry.addr := Cat(ppn, e.getData().addr(corePgIdxBits - 1, 0))
        // printf("New Entry Addr: %x\n", new_entry.addr)
        new_entry.immovable := e.getData().immovable
        new_entry.small := e.getData().small
        new_entry.phys := true.B
        printf("[HTLB] New Entry (tag: %d): %x, %d, %d, %d\n", e.tag, new_entry.addr, new_entry.immovable, new_entry.small, new_entry.phys)

        e.data := new_entry.asUInt
      }
    }
  }

  when(state === s_request) {
    when(io.htw.req.ready) {
      next_state := s_wait
    }
  }

  when(io.htw.resp.valid) {
    next_state := s_ready
  }

  // Send request to L2 HTLB if miss
  io.htw.req.valid := state === s_request
  io.htw.req.bits.valid := true.B
  io.htw.req.bits.bits.hid := r_refill_tag

  // Refill L1 HTLB once L2 HTLB responds
  when(do_refill) {
    val newEntry = Wire(new HTLBEntryData)
    newEntry.phys := false.B
    newEntry.addr := io.htw.resp.bits.hte.addr
    newEntry.immovable := false.B // io.htw.resp.bits.immovable
    newEntry.small := io.htw.resp.bits.hte.small

    midas.targetutils.SynthesizePrintf(printf(
      "[HTLB] New Entry: %x, %d, %d, filling in (tag: %d) \n",
      newEntry.addr,
      newEntry.immovable,
      newEntry.phys,
      r_refill_tag
    ))

    val waddr = Mux(r_sectored_hit, r_sectored_hit_addr, r_sectored_repl_addr)
    for ((e, i) <- entries.zipWithIndex) when(waddr === i.U) {
      // make a copy of the victim entry, and set the victim flag to notify the L2 HTLB
      next_state := Mux(e.valid, s_victim_wait, s_ready)
      victim_entry := e
      e.invalidate()
      e.insert(r_refill_tag, newEntry)
    }
  }

  def replacementEntry(set: Seq[Entry], alt: UInt) = {
    val valids = set.map(_.valid).asUInt
    Mux(valids.andR, alt, PriorityEncoder(~valids))
  }

  when(state === s_victim_wait && io.htw.evict_resp) {
    next_state := s_ready
  }

  when(reset.asBool) {
    entries.foreach(_.invalidate())
  }

  // for (w <- 0 until memWidth) {
  //   for ((e, i) <- entries.zipWithIndex) {
  //     when(e.valid) {
  //       val entry = e.data.asTypeOf(new HTLBEntryData)
  //       printf(
  //         "[HTLB] Entry %d: %d,  %x, %x (%d), %d\n",
  //         i.U,
  //         e.valid,
  //         e.tag,
  //         entry.addr,
  //         entry.phys,
  //         entry.immovable
  //       )
  //     }
  //   }
  // }

  // FSM Logic - get .way from plru, access it, get .way again. Do it until counter === n_ways for hits, go to final state, this marks completion, send resp to commit somehow, and then this is the end of the instruction.

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

  val entry_idx = RegInit(0.U((log2Ceil(entries.size) + 1).W))
  val dumped_entry_idx = RegInit(0.U((log2Ceil(entries.size) + 1).W))
  
  when(io.htDump.orR && state === s_ready) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Starting to dump L1\n"))
    entry_idx := 0.U
    dumped_entry_idx := 0.U
    next_state := s_ht_dump
  }

  when (state === s_ht_dump && entry_idx === entries.size.U) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Done dumping!!!\n"))
    next_state := s_ht_dumped
  }
  
  when (state === s_ht_dumped && !io.htDump.orR) {
    next_state := s_ready
  }

  val d_hid = RegInit(0.U(handleBits.W))
  val hit = WireDefault(false.B)
  when (entry_idx < entries.size.U && state === s_ht_dump) {
    val way = plru.way
    // printf("Entry idx: %d\n", entry_idx)

    hit := entries(way).valid
    when(hit) {
      val entry = entries(way).data.asTypeOf(new HTLBEntryData)
      midas.targetutils.SynthesizePrintf(printf(
        "[HTLB] L1Entry: %d: %d,  %x, %x (%d), %d\n",
        way,
        entries(way).valid,
        entries(way).tag,
        entry.addr,
        entry.phys,
        entry.immovable
      ))
      d_hid := entries(way).tag
      next_state := s_ht_dump_wait
    }
    plru.access(way)
  }

  entry_idx := Mux((state === s_ht_dump && !hit) || (state === s_ht_dump_wait && mem_resp_valid), entry_idx + 1.U, entry_idx);

  io.htw.l1miss := do_refill || htlb_miss.orR
  when(io.htw.l1miss) {
    printf("[HTLB] L1 Miss\n")
  }

  io.mem.keep_clock_enabled := false.B

  val d_hte_vaddr = io.htDump + dumped_entry_idx*8.U

  when (state === s_ht_dump_wait) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Dumping L1 Entry %d to %x\n", d_hid, d_hte_vaddr))
  }

  when (state === s_ht_dump_wait && mem_resp_valid) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Dumped %d-th L1 Entry %d\n", dumped_entry_idx, d_hid))
    dumped_entry_idx := dumped_entry_idx + 1.U
    next_state := s_ht_dump
  }

  io.mem.req.valid := state === s_ht_dump_wait && !mem_resp_valid
  io.mem.req.bits.phys := false.B
  io.mem.req.bits.cmd := M_XWR
  io.mem.req.bits.size := log2Ceil(
    xLen / 8
  ).U // TODO: confirm this makes sense
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.addr := d_hte_vaddr
  io.mem.req.bits.idx.foreach(_ := d_hte_vaddr) // TODO: huh?
  io.mem.req.bits.dprv := PRV.S.U // HTW accesses are S-mode by definition
  io.mem.req.bits.dv := false.B
  io.mem.req.bits.tag := DontCare
  io.mem.req.bits.no_resp := false.B
  io.mem.req.bits.no_alloc := DontCare
  io.mem.req.bits.no_xcpt := DontCare
  io.mem.req.bits.data := d_hid
  io.mem.req.bits.mask := ((1 << coreDataBytes) - 1).U

  // printf("io.mem.req.valid: %d, s1_kill: %d\n", io.mem.req.valid, io.mem.s1_kill)

  io.mem.s1_kill := state =/= s_ht_dump_wait
  io.mem.s1_data.data := d_hid
  io.mem.s1_data.mask := ((1 << coreDataBytes) - 1).U
  io.mem.s2_kill := false.B
}
