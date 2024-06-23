package boom.v4.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import boom.v4.common._
import freechips.rocketchip.tile.CoreBundle

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

case class HTLBConfig(
    nSets: Int,
    nWays: Int
)

class HTLB(cfg: TLBConfig)(implicit p: Parameters) extends BoomModule()(p) {
  val io = IO(new Bundle {
    val req = Flipped(Vec(lsuWidth, Decoupled(new HTLBReq)))
    val resp = Vec(lsuWidth, new HTLBResp)
    val htw = new HTLBHTWIO
    val tlb = Flipped(Vec(lsuWidth, Valid(UInt(ppnBits.W))))
    val sfence = Input(Valid(new SFenceReq))
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

    def ppn(hid: UInt) = {
      getData().addr
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

    def setPAddr(paddr: UInt) = {
      // FIXME: this doesn't feel right? this isn't combinational right?
      val entry = WireDefault(getData())
      entry.addr := paddr
      entry.phys := true.B

      data := entry.asUInt
    }
  }

  // Utilities
  def widthMap[T <: Data](f: Int => T) = VecInit((0 until lsuWidth).map(f))
  val hm_enabled = widthMap(w => !io.req(w).bits.passthrough)
  val hid = widthMap(w => io.req(w).bits.haddr(xLen - 2, handleBits + 1))

  
  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets * cfg.nWays, new Entry()))

  // State Machine
  val s_ready :: s_request :: s_wait :: s_wait_invalidate :: Nil = Enum(4)
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

  val sectored_plru = new PseudoLRU(entries.size)
  for (w <- 0 until lsuWidth) {
    when(io.req(w).valid && hm_enabled(w)) {
      sectored_plru.access(OHToUInt(real_hits(w)))
    }
  }

  // Miss Logic
  val victim = RegInit(false.B)
  val victim_entry = Reg(new Entry())
  val vic = victim_entry.data.asTypeOf(new HTLBEntryData)

  when(io.htw.evict.valid) {
    printf("Victim Entry (%x): %x, %d, %d\n", victim_entry.tag, vic.addr, vic.phys, vic.small)
  }

  io.htw.evict.valid := victim && victim_entry.valid
  io.htw.evict.bits.hid := victim_entry.tag
  io.htw.evict.bits.addr := vic.addr
  io.htw.evict.bits.phys := vic.phys
  io.htw.evict.bits.small := vic.small

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
  for (w <- 0 until lsuWidth) {
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
      addr(w) + io.req(w).bits.haddr(handleBits, 0)
    )
    io.resp(w).phys := phys(w) =/= false.B && hm_enabled(w)
    io.resp(w).small := small(w)

    when(!io.resp(w).miss && !io.req(w).bits.passthrough) {
      printf(
        "[HTLB] -> [LSU] %x %d (for %x) (paddr: %x)\n",
        io.resp(w).addr,
        io.resp(w).phys,
        io.req(w).bits.haddr,
        addr(w) + io.req(w).bits.haddr(handleBits - 1, 0)
      )
    }
  }

  // Finite State Machine Logic
  val sfence = io.sfence.valid
  for (w <- 0 until lsuWidth) {
    when(
      io.req(w).fire && htlb_miss(w) && state === s_ready && !io
        .req(w)
        .bits
        .passthrough
    ) {
      next_state := s_request
      r_refill_tag := hid(w)

      r_sectored_repl_addr := replacementEntry(entries, sectored_plru.way)
      r_sectored_hit_addr := OHToUInt(real_hits(w))
      r_sectored_hit := real_hits(w).orR
    }

    when(io.htw.req.valid) {
      printf("[HTLB] -> [HTW] Looking up hid: %x\n", io.htw.req.bits.bits.hid)
    }

    when(io.tlb(w).valid) {
      /* debug print for small handlen optimization
            printf("paddr for hid %x: %x\n", hid(w), io.tlb(w).bits)
            for ((e, i) <- entries.zipWithIndex) when (e.hit(hid(w))) {
                e.setPAddr(hid(w), io.tlb(w).bits)
            }
       */
    }
  }

  when(state === s_request) {
    when(sfence) { next_state := s_ready }
    when(io.htw.req.ready) {
      next_state := Mux(sfence, s_wait_invalidate, s_wait)
    }
  }

  when(state === s_wait && sfence) {
    next_state := s_wait_invalidate
  }

  when(io.htw.resp.valid) {
    next_state := s_ready
  }

  // TODO: figure out sfence here
  when(sfence) {
    for (w <- 0 until lsuWidth) {
      // TODO: add some assertion here?
      // TODO: what invalidations do we need to support? 1. individual hid, 2. all, (optional 3. non-global but not a problem now)
      // assert(!io.sfence.bits.rs1 || (io.sfence.bits.addr >> pgIdxBits) === vpn(w))
      for (e <- entries) {
        when(io.sfence.bits.rs1) {
          e.invalidate()
        }.otherwise {
          e.invalidate()
        }
      }
    }
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

    printf(
      "New Entry: %x, %d, %d, filling in (tag: %d) \n",
      newEntry.addr,
      newEntry.immovable,
      newEntry.phys,
      r_refill_tag
    )

    val waddr = Mux(r_sectored_hit, r_sectored_hit_addr, r_sectored_repl_addr)
    for ((e, i) <- entries.zipWithIndex) when(waddr === i.U) {
      // make a copy of the victim entry, and set the victim flag to notify the L2 HTLB
      victim := true.B
      victim_entry := RegNext(e)
      e.invalidate()
      e.insert(r_refill_tag, newEntry)
    }
  }

  def replacementEntry(set: Seq[Entry], alt: UInt) = {
    val valids = set.map(_.valid).asUInt
    Mux(valids.andR, alt, PriorityEncoder(~valids))
  }

  when(reset.asBool) {
    entries.foreach(_.invalidate())
  }

  for (w <- 0 until lsuWidth) {
    for ((e, i) <- entries.zipWithIndex) {
      when(e.valid) {
        val entry = e.data.asTypeOf(new HTLBEntryData)
        printf(
          "Entry %d: %d,  %x, %x (%d), %d\n",
          i.U,
          e.valid,
          e.tag,
          entry.addr,
          entry.phys,
          entry.immovable
        )
      }
    }
  }
}
