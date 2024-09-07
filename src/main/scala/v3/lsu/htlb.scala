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
    val htInval = Input(UInt(handleBits.W))
    val htBase = Input(UInt(maxSVAddrBits.W))
    val sfence = Input(Valid(new SFenceReq))
    val kill = Input(Bool())
    val ptw_access = Input(Bool())
  })

  class HTLBEntryData() extends Bundle() {
    val phys = Bool()
    val addr = UInt(xLen.W)
    // val immovable = Bool()
    val small = Bool()
  }
  class Entry(nSets: Int) extends Bundle {
    val idxBits = log2Ceil(nSets)
    val tagBits = handleBits - idxBits
    val tag = UInt(tagBits.W)
    val data = UInt(new HTLBEntryData().getWidth.W)
    val valid = Bool()

    def getData() = data.asTypeOf(new HTLBEntryData)
    def hit(tag: UInt) = {
      valid && this.tag === tag
    }

    def insert(tag: UInt, entry: HTLBEntryData) = {
      this.tag := tag

      valid := true.B
      data := entry.asUInt
    }

    def invalidate() = { valid := false.B }

    // def lock(hid: UInt) = {
    //   getData().immovable := true.B
    // }

    // def unlock(hid: UInt) = {
    //   getData().immovable := false.B
    // }
  }

  // Utilities
  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))
  val hm_enabled = widthMap(w => !io.req(w).bits.passthrough)
  val hid = widthMap(w => io.req(w).bits.haddr(xLen - 2, handleOffsetBits))
  val idxBits = log2Ceil(cfg.nSets)

  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets, Vec(cfg.nWays, new Entry(cfg.nSets))))

  // State Machine
  val s_ready :: s_request :: s_wait :: s_wait_invalidate :: s_victim_wait :: s_dumping :: s_dumping_wait :: s_dumped :: Nil = Enum(8)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)
  state := Mux(io.htBase.orR, OptimizationBarrier(next_state), s_ready)

  // Refill State
  val do_refill = io.htw.resp.valid
  val r_refill_hid = Reg(UInt(handleBits.W))
  // val r_sectored_repl_addr = Reg(UInt(log2Ceil(entries.size).W))
  // val r_sectored_hit_addr = Reg(UInt(log2Ceil(entries.size).W))
  // val r_sectored_hit = Reg(Bool())

  val l1_plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")

  // Hit Logic
  val hid_tag = widthMap(w => Split(hid(w), idxBits)._1)
  val hid_set = widthMap(w => Split(hid(w), idxBits)._2)
  val htlb_hit = Wire(Vec(memWidth, Bool()))
  val htlb_miss = Wire(Vec(memWidth, Bool()))
  val real_hits = Wire(Vec(memWidth, UInt(cfg.nWays.W)))
  val hitVec = Wire(Vec(memWidth, Vec(cfg.nWays, Bool())))
  for (w <- 0 until memWidth) {
  //   printf("w: %d, hid: %x, hid_tag: %x, hid_set: %x\n", w.U, hid(w), hid_tag(w), hid_set(w))
    hitVec(w) := entries(hid_set(w)).map(hm_enabled(w) && _.hit(hid_tag(w)))
  //   printf("hitVec: %x\n", hitVec(w).asUInt)
    real_hits(w) := hitVec(w).asUInt
  //   printf("real_hits: %x\n", real_hits(w))
    htlb_hit(w) := real_hits(w).orR
  //   printf("htlb_hit: %x\n", htlb_hit(w))
    htlb_miss(w) := hm_enabled(w) && !htlb_hit(w)
  //   printf("htlb_miss: %x\n", htlb_miss(w))
  }

  // for (s <- 0 until cfg.nSets) {
  //   printf("[HTLB] Set %d\n", s.U)
  //   for (w <- 0 until cfg.nWays) {
  //     when(entries(s)(w).valid) {
  //       val entry = entries(s)(w).data.asTypeOf(new HTLBEntryData)
  //       printf(
  //         "[HTLB] Entry %d: %d,  %x, %x (%d)\n",
  //         w.U,
  //         entries(s)(w).valid,
  //         entries(s)(w).tag,
  //         entry.addr,
  //         entry.phys
  //       )
  //     }
  //   }
  // }

  /*
  val hitsVec =
    widthMap(w => VecInit(entries.map(hm_enabled(w) && _.hit(hid(w)))))
  val real_hits = widthMap(w => hitsVec(w).asUInt)
  val htlb_hit = widthMap(w => real_hits(w).orR)
  val htlb_miss = widthMap(w => hm_enabled(w) && !htlb_hit(w))
  */

  for (w <- 0 until memWidth) {
    when(io.req(w).valid && hm_enabled(w)) {
      l1_plru.access(hid_set(w), OH1ToUInt(real_hits(w)))
    }
  }

  // Miss Logic
  // val victim_ready :: victim_wait :: Nil = Enum(2)
  // val victim = RegInit(victim_ready)
  val victim_entry = Reg(new Entry(cfg.nSets))
  val vic = victim_entry.data.asTypeOf(new HTLBEntryData)

  when(io.htw.evict.valid) {
    midas.targetutils.SynthesizePrintf(printf(
      "[HTLB] Victim Entry (%x): %x\n",
      victim_entry.tag,
      vic.addr
    ))
  }

  io.htw.evict.valid := state === s_victim_wait && !io.htw.evict_resp
  io.htw.evict.bits.hid := victim_entry.tag
  io.htw.evict.bits.addr := vic.addr
  io.htw.evict.bits.phys := vic.phys
  io.htw.evict.bits.small := vic.small
  io.htw.l1_dumped := state === s_dumped

  // Utilities to help respond
  val addr = widthMap(w =>
    entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData).addr)
  val phys = widthMap(w =>
    entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData).phys)
  val small = widthMap(w =>
    entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData).small)

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
    io.resp(w).addr := Mux(io.req(w).bits.passthrough, effective_address,
                          Mux (!io.resp(w).miss, addr(w) + io.req(w).bits.haddr(handleBits - 1, 0), 0.U))
    io.resp(w).phys := false.B
    io.resp(w).small := false.B

    when(!io.resp(w).miss && io.req(w).valid && !io.req(w).bits.passthrough) {
       midas.targetutils.SynthesizePrintf(printf(
        "[HTLB] -> [LSU] %x %d (for %x) (paddr: %x)\n",
        io.resp(w).addr,
        io.resp(w).phys,
        io.req(w).bits.haddr,
        addr(w) + io.req(w).bits.haddr(handleOffsetBits - 1, 0)
      ))
    }
  }

  val sfence = io.sfence.valid
  for (w <- 0 until memWidth) {
    when(
      io.req(w).fire && htlb_miss(w) && state === s_ready && !io
        .req(w)
        .bits
        .passthrough
    ) {
      state := s_request
      r_refill_hid := hid(w)
    }

    when(io.htw.req.valid) {
      midas.targetutils.SynthesizePrintf(printf("[HTLB] -> [HTW] Looking up hid: %x\n", io.htw.req.bits.bits.hid))
    }

    // TODO: add paddr optimization back in
    // when(io.tlb(w).valid) {
    //   // debug print for small handlen optimization
    //   printf("[HTLB] paddr for hid %x: %x\n", io.tlb(w).bits.hid(xLen - 2, handleBits), io.tlb(w).bits.paddr)
    //   for ((e, i) <- entries.zipWithIndex) when(e.hit(io.tlb(w).bits.hid(xLen - 2, handleBits))) {
    //     // e.setPAddr(io.tlb(w).bits.paddr)

    //     val ppn = io.tlb(w).bits.paddr
    //     val new_entry = Wire(new HTLBEntryData())
    //     new_entry.addr := Cat(ppn, e.getData().addr(corePgIdxBits - 1, 0))
    //     // printf("New Entry Addr: %x\n", new_entry.addr)
    //     new_entry.immovable := e.getData().immovable
    //     new_entry.small := e.getData().small
    //     new_entry.phys := true.B
    //     printf("[HTLB] New Entry (tag: %d): %x, %d, %d, %d\n", e.tag, new_entry.addr, new_entry.immovable, new_entry.small, new_entry.phys)

    //     e.data := new_entry.asUInt
    //   }
    // }
  }

  when(state === s_request) {
    when (sfence) { next_state := s_ready }
    when (io.htw.req.ready) { next_state := Mux(sfence, s_wait_invalidate, s_wait) }
  }
  when (state === s_wait && sfence) {
    next_state := s_wait_invalidate
  }
  when (state === s_wait_invalidate) {
    next_state := s_ready
  }
  when(io.htw.resp.valid) {
    next_state := s_ready
  }

  when (sfence) {
    entries.foreach(_.foreach(_.invalidate()))
  }

  // Send request to L2 HTLB if miss
  io.htw.req.valid := state === s_request
  io.htw.req.bits.valid := !io.kill
  io.htw.req.bits.bits.hid := r_refill_hid

  // Refill L1 HTLB once L2 HTLB responds
  when(do_refill) {
    val newEntry = Wire(new HTLBEntryData)
    newEntry.phys := false.B
    newEntry.addr := io.htw.resp.bits.hte.addr
    // newEntry.immovable := false.B // io.htw.resp.bits.immovable
    newEntry.small := io.htw.resp.bits.hte.small

    midas.targetutils.SynthesizePrintf(printf(
      "[HTLB] New Entry: %x, %d, filling in (tag: %d) \n",
      newEntry.addr,
      // newEntry.immovable,
      newEntry.phys,
      r_refill_hid
    ))

    val (r_tag, r_idx) = Split(r_refill_hid, idxBits)
    val repl_way = l1_plru.way(r_idx)
    // printf("[HTLB] Replacing way: %d, with tag: %d in set: %d\n", repl_way, r_tag, r_idx)
    val e = entries(r_idx)(repl_way)
    // printf("[HTLB] Replacing Entry: Valid: %d, Tag: %d, Addr: %x, Phys: %d\n", e.valid, e.tag, e.getData().addr, e.getData().phys)
    // make a copy of the victim entry, and set the victim flag to notify the L2 HTLB
    state := Mux(e.valid, s_victim_wait, s_ready)
    victim_entry := e
    // e.invalidate()
    e.insert(r_tag, newEntry)
  }

  when(state === s_victim_wait && io.htw.evict_resp) {
    next_state := s_ready
  }

  when(reset.asBool) {
    entries.foreach(_.foreach(_.invalidate()))
  }

  when (io.htInval.orR) {
    when (io.htInval === ((BigInt(1) << handleBits) - 1).U) {
      midas.targetutils.SynthesizePrintf(printf("[HTW] Invalidating all entries\n"))
      entries.foreach(_.foreach(_.invalidate()))
    }.otherwise {
      midas.targetutils.SynthesizePrintf(printf("[HTLB] Invalidating %x\n", io.htInval))
      val (e_tag, e_idx) = Split(io.htInval, idxBits)
      for (e <- entries(e_idx)) {
        when(e_tag === e.tag) {
          e.invalidate()
        }
      }
    }
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

  val set_idx = RegInit(0.U((log2Ceil(cfg.nSets) + 1).W))
  val way_idx = RegInit(0.U((log2Ceil(cfg.nWays) + 1).W))
  val dumped_entry_idx = RegInit(0.U((log2Ceil(entries.size) + 1).W))
  
  when(io.htDump.orR && state === s_ready) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Starting to dump L1\n"))
    set_idx := 0.U
    way_idx := 0.U
    dumped_entry_idx := 0.U
    state := s_dumping
  }

  when (state === s_dumping && set_idx === cfg.nSets.U && way_idx === cfg.nWays.U) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Done dumping!!!\n"))
    state := s_dumped
  }
  
  when (state === s_dumped && !io.htDump.orR) {
    state := s_ready
  }

  val d_hid = RegInit(0.U(handleBits.W))
  val hit = WireDefault(false.B)

  val way_idx_update = (state === s_dumping && !hit) || (state === s_dumping_wait && mem_resp_valid)
  val set_idx_update = way_idx === cfg.nWays.U && state === s_dumping

  val way_clear = (set_idx_update || (state === s_dumping_wait && way_idx === coreParams.nL2TLBWays.U)) || (!io.htDump.orR && state === s_dumping)
  val set_clear = !io.htDump.orR && state === s_dumping

  when (set_idx < cfg.nSets.U && way_idx < cfg.nWays.U && state === s_dumping) {
    val way = l1_plru.way(set_idx(idxBits-1,0))

    hit := entries(set_idx(idxBits-1,0))(way).valid
    when(hit) {
      val entry = entries(set_idx(idxBits-1,0))(way).data.asTypeOf(new HTLBEntryData)
      midas.targetutils.SynthesizePrintf(printf(
        "[HTLB] L1Entry: %d: %d,  %x, %x (%d)\n",
        way,
        entries(set_idx(idxBits-1,0))(way).valid,
        entries(set_idx(idxBits-1,0))(way).tag,
        entry.addr,
        entry.phys,
        // entry.immovable
      ))
      d_hid := Cat(entries(set_idx(idxBits-1,0))(way).tag, set_idx(idxBits - 1, 0))
      state := s_dumping_wait
    }
    l1_plru.access(set_idx(idxBits-1,0), way)
  }

  way_idx := Mux(way_idx_update, way_idx + 1.U, Mux(way_clear, 0.U, way_idx))
  set_idx := Mux(set_idx_update, set_idx + 1.U, 
                     Mux(state === s_dumping_wait && way_idx === coreParams.nL2TLBWays.U, d_hid(idxBits-1,0) + 1.U, 
                         Mux(set_clear, 0.U, set_idx)))



  io.htw.l1miss := do_refill || htlb_miss.orR
  when(io.htw.l1miss) {
    printf("[HTLB] L1 Miss\n")
  }

  io.mem.keep_clock_enabled := false.B

  val d_hte_vaddr = io.htDump + dumped_entry_idx*8.U

  when (state === s_dumping_wait) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Dumping L1 Entry %d to %x\n", d_hid, d_hte_vaddr))
  }

  when (state === s_dumping_wait && mem_resp_valid) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Dumped %d-th L1 Entry %d\n", dumped_entry_idx, d_hid))
    dumped_entry_idx := dumped_entry_idx + 1.U
    state := s_dumping
  }

  io.mem.req.valid := state === s_dumping_wait && !mem_resp_valid
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
  val replay_htlb_dump_req = io.ptw_access && io.mem.req.valid
  when (replay_htlb_dump_req) {
    midas.targetutils.SynthesizePrintf(printf("[HTLB] Replaying HTLB Dump\n"))
  }

  io.mem.s1_kill := state =/= s_dumping_wait || replay_htlb_dump_req
  io.mem.s1_data.data := d_hid
  io.mem.s1_data.mask := ((1 << coreDataBytes) - 1).U
  io.mem.s2_kill := replay_htlb_dump_req
}
