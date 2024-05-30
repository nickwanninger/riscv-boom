package boom.v4.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import boom.v4.common._

class HTLBReq(implicit p: Parameters) extends BoomBundle()(p)
{
    val haddr = UInt(xLen.W)
    val passthrough = Bool()
    val sfence = Input(Valid(new SFenceReq))
}

class HTLBResp(implicit p: Parameters) extends BoomBundle()(p)
{
    val addr = UInt(xLen.W)
    // lookup responses
    val miss = Bool()
    // addr is physical
    val phys = Bool()
    // TODO: Add handle related exceptions
}

case class HTLBConfig(
    nSets: Int,
    nWays: Int,
)

class HTLB(cfg: TLBConfig)(implicit p: Parameters) extends BoomModule()(p)
{
    val io = IO(new Bundle {
        val req = Flipped(Vec(lsuWidth, Decoupled(new HTLBReq)))
        val miss_rdy = Output(Bool())
        val resp = Vec(lsuWidth, new HTLBResp)
        val htw = new HTLBHTWIO
        val kill = Input(Bool())
        val tlb = Flipped(Vec(lsuWidth, Decoupled(UInt(ppnBits.W))))
        val hasid = Input(UInt(asIdBits max 1).W)
    })

    class EntryData() extends Bundle() {
        val vaddr = UInt(xLen.W)
        val temporal_order = UInt(log2Ceil(cfg.nSets * cfg.nWays).W)
        val immovable = Bool()
        val counter = UInt(8.W)
        val ppn = UInt(ppnBits.W)
        val hasid = UInt((asIdBits max 1).W) // TODO zero-width
    }

    class Entry(val nSectors: Int) extends Bundle {
        require(isPow2(nSectors))
        require(nSectors == 1) // FIXME: this is a hack to make the code work for now

        val tag = UInt(handleBits.W)
        val data = Vec(nSectors, UInt(new EntryData().getWidth.W))
        val valid = Vec(nSectors, Bool())
        def entry_data = data.map(_.asTypeOf(new EntryData))
        val hits = Vec(nSectors, UInt(8.W))

        private def sectorIdx(hid: UInt) = hid.extract(log2Ceil(nSectors)-1, 0)
        def getData(hid: UInt) = data(sectorIdx(hid)).asTypeOf(new EntryData)
        def sectorHit(hid: UInt) = valid.orR && sectorTagMatch(hid)
        def sectorTagMatch(hid: UInt) = ((this.tag ^ hid) >> log2Ceil(nSectors)) === 0.U
        def hit(hid: UInt) = {
            val idx = sectorIdx(hid)

            val did_hit = valid(idx) && sectorTagMatch(hid) 

            when (did_hit) {
                val entry = getData(hid)
                hits(idx) := Mux(hits(idx) =/= 255.U, hits(idx) + 1.U, hits(idx))
                entry.counter := hits(idx)
                data(idx) := entry.asUInt
            }

            did_hit
        }

        def ppn(hid: UInt) = getData(hid).ppn

        def insert(hid: UInt, entry: EntryData) = {
            this.tag := hid

            val idx = sectorIdx(hid)
            valid(idx) := true.B
            data(idx) := entry.asUInt
        }

        def invalidate() = { valid.foreach(_ := false.B) }

        def invalidateHASID() = { 
            // TODO: fix to actually read hasid
            valid.foreach(_ := false.B)
        }

        def lock(hid: UInt) = {
            getData(hid).immovable := true.B
        }

        def unlock(hid: UInt) = {
            getData(hid).immovable := false.B
        }

        def setPPN(hid: UInt, ppn: UInt) = {
            val entry = getData(hid)
            entry.ppn := ppn

            val idx = sectorIdx(hid)
            data(idx) := entry.asUInt
        }
    }

    val counter = RegInit(0.U(log2Ceil(cfg.nSets * cfg.nWays).W))

    val vm_enabled = widthMap(w => !io.req(w).bits.passthrough)

    val entries = Reg(Vec(cfg.nSets * cfg.nWays, new Entry(1)))
    val r_sectored_repl_addr = Reg(UInt(log2Ceil(entries.size).W))
    val r_sectored_hit_addr = Reg(UInt(log2Ceil(entries.size).W))
    val r_sectored_hit = Reg(Bool())

    val s_ready :: s_request :: s_wait :: s_wait_invalidate :: Nil = Enum(4)
    val state = RegInit(s_ready)

    val do_refill = io.htw.resp.valid

    def widthMap[T <: Data](f: Int => T) = VecInit((0 until lsuWidth).map(f))

    val r_refill_tag = Reg(UInt(handleBits.W))

    val hid = widthMap(w => io.req(w).bits.haddr(xLen-2, handleBits))
    val hitsVec = widthMap(w => VecInit(entries.map(vm_enabled(w) && _.hit(hid(w)))))
    val real_hits = widthMap(w => hitsVec(w).asUInt)
    val hits = widthMap(w => Cat(!vm_enabled(w), real_hits(w)))

    val vaddr = widthMap(w => Mux1H(hitsVec(w) :+ !vm_enabled(w), entries.map(_.data.asTypeOf(new EntryData).vaddr)))
    val ppn = widthMap(w => Mux1H(hitsVec(w) :+ !vm_enabled(w), entries.map(_.data.asTypeOf(new EntryData).ppn)))
    val paddr = widthMap(w => Cat(ppn(w), vaddr(w)(corePgIdxBits-1,0)))

    for (w <- 0 until lsuWidth) {
        for ((e, i) <- entries.zipWithIndex) {
            for (s <- 0 until e.nSectors) {
                when (e.valid(s)) {
                    val entry = e.data(s).asTypeOf(new EntryData)
                    printf("Entry %d: %d, %d, %x, %d, %d (%x), counter: %d\n", i.U, e.valid(s), e.tag, entry.vaddr, entry.immovable, entry.temporal_order, entry.ppn, entry.counter)
                }
            }
        }
    }

    val htlb_hit = widthMap(w => real_hits(w).orR)
    val htlb_miss = widthMap(w => vm_enabled(w) && !htlb_hit(w))

    val sector_hits = widthMap(w => VecInit(entries.map(_.sectorHit(hid(w)))))

    val sectored_plru = new PseudoLRU(entries.size)
    for (w <- 0 until lsuWidth) {
        when (io.req(w).valid && vm_enabled(w)) {
          when (sector_hits(w).orR) { sectored_plru.access(OHToUInt(sector_hits(w))) }
        }
      }

    io.miss_rdy := state === s_ready

    for (w <- 0 until lsuWidth) {
        // handle original case
        val sum = io.req(w).bits.haddr
        val ea_sign = Mux(sum(vaddrBits-1), ~sum(63,vaddrBits) === 0.U,
                                             sum(63,vaddrBits) =/= 0.U)
        val effective_address = Cat(ea_sign, sum(vaddrBits-1,0)).asUInt

        io.req(w).ready  := true.B
        io.resp(w).miss  := do_refill || htlb_miss(w)
        io.resp(w).addr  := Mux(io.req(w).bits.passthrough, effective_address, Mux(ppn(w) === 0.U, vaddr(w) + io.req(w).bits.haddr(handleBits-1,0), paddr(w) + io.req(w).bits.haddr(handleBits-1,0)))
        // io.resp(w).addr  := Mux(io.req(w).bits.passthrough, effective_address, vaddr(w) + io.req(w).bits.haddr(handleBits-1,0))
        io.resp(w).phys  := ppn(w) =/= 0.U && vm_enabled(w)

        when (!io.resp(w).miss && !io.req(w).bits.passthrough) {
          printf("[HTLB] -> [LSU] %x %d (for %x) (paddr: %x, vaddr: %x)\n", io.resp(w).addr, io.resp(w).phys, io.req(w).bits.haddr, paddr(w) + io.req(w).bits.haddr(handleBits-1,0), vaddr(w) + io.req(w).bits.haddr(handleBits-1,0))
        }
    }

    io.htw.req.valid := state === s_request
    io.htw.req.bits.valid := !io.kill
    io.htw.req.bits.bits.hid := r_refill_tag

    when (do_refill) {
        val newEntry = Wire(new EntryData)
        newEntry.vaddr := io.htw.resp.bits.addr
        newEntry.immovable := false.B // TODO: should be pulled from entry probably
        newEntry.temporal_order := counter
        newEntry.ppn := 0.U
        newEntry.counter := 0.U
        newEntry.hasid := io.hasid

        printf("New Entry: %x, %d, %d, %x\n", newEntry.vaddr, newEntry.immovable, newEntry.temporal_order, newEntry.ppn)

        val waddr = Mux(r_sectored_hit, r_sectored_hit_addr, r_sectored_repl_addr)
        for ((e, i) <- entries.zipWithIndex) when (waddr === i.U) {
            e.invalidate()
            e.insert(r_refill_tag, newEntry)
        }
        counter := Mux(counter === (cfg.nSets * cfg.nWays - 1).U, 0.U, counter + 1.U)
    }

    val sfence = io.sfence.valid
    for (w <- 0 until lsuWidth) {
        // printf("[HTLB] State: %d, %d, %d, %d (hid: %d)\n", io.req(w).fire, htlb_miss(w), state, !io.req(w).bits.passthrough, hid(w))
        when (io.req(w).fire && htlb_miss(w) && state === s_ready && !io.req(w).bits.passthrough) {
          state := s_request
          r_refill_tag := hid(w)

          r_sectored_repl_addr  := replacementEntry(entries, sectored_plru.way)
          r_sectored_hit_addr   := OHToUInt(sector_hits(w))
          r_sectored_hit        := sector_hits(w).orR
        }

        when (io.htw.req.valid) {
          printf("[HTLB] -> [HTW] Looking up hid: %x\n", io.htw.req.bits.bits.hid)
        }

        io.tlb(w).ready := true.B
        when (io.tlb(w).valid) {
            printf("paddr for hid %x: %x\n", hid(w), io.tlb(w).bits)
            // TODO: add check here to see if this is valid (i.e., not falling off a page)
            for ((e, i) <- entries.zipWithIndex) when (e.hit(hid(w))) {
                e.setPPN(hid(w), io.tlb(w).bits)
            }
        }
    }

    when (state === s_request) {
      when (sfence) { state := s_ready }
      when (io.htw.req.ready) { state := Mux(sfence, s_wait_invalidate, s_wait) }
      when (io.kill) { state := s_ready }
    }

    when (state === s_wait && sfence) {
      state := s_wait_invalidate
    }
    when (io.htw.resp.valid) {
      state := s_ready
    }

    when (sfence) {
      for (w <- 0 until lsuWidth) {
        // TODO: add some assertion here?
        // assert(!io.sfence.bits.rs1 || (io.sfence.bits.addr >> pgIdxBits) === vpn(w))
        for (e <- all_entries) {
          when (io.sfence.bits.rs3) {
            when (io.sfence.bits.rs1) {
                e.invalidate()
            } .otherwise {
                e.invalidateHASID(io.sfence.bits.asid)
            }
          } .otherwise {}
        }
      }
    }

    when (io.htw.resp.valid) {
      state := s_ready
    }

    def replacementEntry(set: Seq[Entry], alt: UInt) = {
        val valids = set.map(_.valid).asUInt
        Mux(valids.andR, alt, PriorityEncoder(~valids))
      }

    when (reset.asBool) {
      entries.foreach(_.invalidate())
    }
}

/*
class HArbIO(implicit p: Parameters) extends BoomBundle()(p)
{
    val exe = Vec(memWidth, new LSUExeIO)
    val lsu = Flipped(Vec(memWidth, new LSUExeIO))
}

class HArb(harbDelay: Int, hteWidth: Int)(implicit p: Parameters) extends BoomModule()(p)
{
    val io = IO(new HArbIO)

    /*
    * We need to ensure that we comply with the RVWMO model.
    *
    * If we allow handle addresses and regular virtual addresses to be pushed
    * into the LSU OoO, then we need to ensure that handle addresses translations
    * for the same address are not re-ordered with respect to each other. This
    * can be prevented by assuming that handle address translations are
    * effectively hashed wrt to the handle id, which ensures that the
    * serialization on the available ports are correct.
    *
    * Obviously, we maintain the expected behavior wrt sfences.
    */

    for (w <- 0 until memWidth) {
        io.lsu(w).req.bits <> io.exe(w).req.bits

        when (io.exe(w).req.bits.addr(63)) {
            // Construct request to htlb
            val hid = io.exe(w).req.bits.addr(62, 32)

            // Recv response from htlb
            val addr = "hffff_abcd_0000_0000".U + hid * hteWidth.U

            io.lsu(w).req.bits.addr := addr
            printf("Translating hid: 0x%x to haddr: 0x%x\n", hid, io.lsu(w).req.bits.addr)
            io.lsu(w).req.valid := ShiftRegister(io.exe(w).req.valid, harbDelay)
         } .otherwise {
            val ea_sign = Mux(io.exe(w).req.bits.addr(vaddrBits-1), ~io.exe(w).req.bits.addr(63,vaddrBits) === 0.U,
                                                  io.exe(w).req.bits.addr(63,vaddrBits) =/= 0.U)
            val addr = Cat(ea_sign, io.exe(w).req.bits.addr(vaddrBits-1,0)).asUInt

            io.lsu(w).req.bits.addr := addr
            io.lsu(w).req.valid := io.exe(w).req.valid
        }

        io.exe(w).iresp <> io.lsu(w).iresp
        io.exe(w).fresp <> io.lsu(w).fresp
    }
}
*/
