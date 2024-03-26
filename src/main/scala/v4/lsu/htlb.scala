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

class HTLB(cfg: HTLBConfig)(implicit p: Parameters) extends BoomModule()(p)
{
    val io = IO(new Bundle {
        val req = Flipped(Vec(lsuWidth, Decoupled(new HTLBReq)))
        val miss_rdy = Output(Bool())
        val resp = Vec(lsuWidth, new HTLBResp)
        val htw = new HTLBHTWIO
        val kill = Input(Bool())
    })

    class EntryData() extends Bundle() {
        val addr = UInt(xLen.W)
        val immovable = Bool()
        val temporal_order = UInt(log2Ceil(cfg.nSets * cfg.nWays).W)
    }

    class Entry() extends Bundle {
        val tag = UInt(handleBits.W)
        val data = UInt(new EntryData().getWidth.W)
        val valid = Bool()

        def entry_data = data.asTypeOf(new EntryData)
        def getData(vpn: UInt) = OptimizationBarrier(data.asTypeOf(new EntryData))

        def sectorHit(hid: UInt) = valid
        def invalidate() = { valid := false.B }

        def hit(hid: UInt) = {
            valid && hid === tag
        }

        def insert(hid: UInt, entry: EntryData) = {
            this.tag := hid
            this.data := entry.asUInt
            this.valid := true.B
        }

        def lock() = {
            this.data.asTypeOf(new EntryData).immovable := true.B
        }

        def unlock() = {
            this.data.asTypeOf(new EntryData).immovable := false.B
        }
    }

    val counter = RegInit(0.U(log2Ceil(cfg.nSets * cfg.nWays).W))

    val vm_enabled = widthMap(w => !io.req(w).bits.passthrough)

    val entries = Reg(Vec(cfg.nSets * cfg.nWays, new Entry))
    val r_sectored_repl_addr = Reg(UInt(log2Ceil(entries.size).W))

    val s_ready :: s_request :: s_wait :: s_wait_invalidate :: Nil = Enum(4)
    val state = RegInit(s_ready)


    val do_refill = io.htw.resp.valid

    def widthMap[T <: Data](f: Int => T) = VecInit((0 until lsuWidth).map(f))

    val r_refill_tag = Reg(UInt(handleBits.W))

    val hid = widthMap(w => io.req(w).bits.haddr(xLen-2, handleBits))
    // val hitsVec = widthMap(w => VecInit(entries.map(_.hit(hid(w)))))
    val hitsVec = widthMap(w => VecInit(entries.map(vm_enabled(w) && _.hit(hid(w)))))
    val real_hits = widthMap(w => hitsVec(w).asUInt)
    // for (w <- 0 until lsuWidth) {
    //     printf("real_hits: %x, %x\n", real_hits(w), entries(0).hit(hid(w)))
    // }
    val hits = widthMap(w => Cat(!vm_enabled(w), real_hits(w)))

    val ppn = widthMap(w => Mux1H(hitsVec(w) :+ !vm_enabled(w), entries.map(_.data.asTypeOf(new EntryData).addr)))

    for ((e, i) <- entries.zipWithIndex) {
        when (e.valid) {
            printf("Entry %d: %d, %d, %x, %d, %d\n", i.U, e.valid, e.tag, e.data.asTypeOf(new EntryData).addr, e.data.asTypeOf(new EntryData).immovable, e.asTypeOf(new EntryData).temporal_order)
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
        io.resp(w).addr  := Mux(io.req(w).bits.passthrough, effective_address, ppn(w) + io.req(w).bits.haddr(handleBits-1,0))
        io.resp(w).phys  := false.B

        // printf("vaddr req: %x, passthrough %d\n", io.req(w).bits.haddr, io.req(w).bits.passthrough)
        // printf("%d, %d, %d\n", do_refill, htlb_miss(w), htlb_hit(w))
        when (!io.resp(w).miss && !io.req(w).bits.passthrough) {
        //   printf("ppn: %x (%d), is_handle: %d\n", ppn(w) + io.req(w).bits.haddr(31, 0), io.req(w).bits.passthrough, io.req(w).bits.haddr(63) && !io.req(w).bits.haddr(62))
          printf("[HTLB] -> [LSU] %x %d\n", io.resp(w).addr, io.resp(w).phys)
        }
    }

    io.htw.req.valid := state === s_request
    io.htw.req.bits.valid := !io.kill
    io.htw.req.bits.bits.hid := r_refill_tag

    when (do_refill) {
        val newEntry = Wire(new EntryData)
        newEntry.addr := io.htw.resp.bits.addr
        newEntry.immovable := false.B // TODO: should be pulled from entry probably
        newEntry.temporal_order := counter

        for ((e, i) <- entries.zipWithIndex) when (r_sectored_repl_addr === i.U) {
            e.invalidate()
            counter := Mux(counter === (cfg.nSets * cfg.nWays - 1).U, 0.U, counter + 1.U)
            e.insert(r_refill_tag, newEntry)
        }
    }

    for (w <- 0 until lsuWidth) {
        // printf("[HTLB] State: %d, %d, %d, %d\n", io.req(w).fire, htlb_miss(w), state, !io.req(w).bits.passthrough);
        when (io.req(w).fire && htlb_miss(w) && state === s_ready && !io.req(w).bits.passthrough) {
          state := s_request
          r_refill_tag := hid(w)

          r_sectored_repl_addr  := replacementEntry(entries, sectored_plru.way)
        }

        when (io.htw.req.valid) {
          printf("[HTLB] -> [HTW] Looking up hid: %x\n", io.htw.req.bits.bits.hid)
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
