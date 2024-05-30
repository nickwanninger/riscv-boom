package boom.v4.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.constants._
import freechips.rocketchip.rocket.HellaCacheIO

import boom.v4.common._

class HTWReq(implicit p: Parameters) extends BoomBundle()(p)
{
    val hid = UInt(32.W)
}

class HTWResp(implicit p: Parameters) extends BoomBundle()(p)
{
    val addr = UInt(xLen.W)
}

class HTLBHTWIO(implicit p: Parameters) extends BoomBundle()(p) {
  val req = Decoupled(Valid(new HTWReq))
  val resp = Flipped(Valid(new HTWResp))
}

class DatapathHTWIO(implicit p: Parameters) extends BoomBundle()(p)
{
    val htBase = Input(UInt(xLen.W))
}

class HTW(implicit p: Parameters) extends BoomModule()(p) {
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

    val s_ready :: s_req :: Nil = Enum(2)
    val state = RegInit(s_ready)
    val next_state = WireDefault(state)
    state := OptimizationBarrier(next_state)

    val hte_vaddr = io.dpath.htBase + io.requestor.req.bits.bits.hid * 8.U
    // val ea_sign = Mux(sum(vaddrBits-1), ~sum(63,vaddrBits) === 0.U,
    //                                      sum(63,vaddrBits) =/= 0.U)
    // val effective_address = Cat(ea_sign, sum(vaddrBits-1,0)).asUInt

    // mem request
    io.mem.keep_clock_enabled := false.B

    io.mem.req.valid := state === s_req
    io.mem.req.bits.phys := false.B
    io.mem.req.bits.cmd  := M_XRD
    io.mem.req.bits.size := log2Ceil(xLen/8).U // 2.U
    printf("[HTW] -> [Mem] Requesting HID %d at %x (%d)\n", io.requestor.req.bits.bits.hid, hte_vaddr, io.mem.req.bits.size)
    io.mem.req.bits.signed := false.B
    io.mem.req.bits.addr := hte_vaddr
    io.mem.req.bits.idx.foreach(_ := hte_vaddr) // huh?
    io.mem.req.bits.dprv := PRV.S.U   // HTW accesses are S-mode by definition
    io.mem.req.bits.dv := false.B
    io.mem.req.bits.tag := DontCare
    io.mem.req.bits.no_resp := false.B
    io.mem.req.bits.no_alloc := DontCare
    io.mem.req.bits.no_xcpt := DontCare
    io.mem.req.bits.data := DontCare
    io.mem.req.bits.mask := DontCare

    io.mem.s1_kill := false.B //l2_hit || state =/= s_wait1
    io.mem.s1_data := DontCare
    io.mem.s2_kill := false.B

    when (io.mem.req.valid) {
        printf("[HTW] -> [Mem] Looking up HID: %d at %x\n", io.requestor.req.bits.bits.hid, io.mem.req.bits.addr)
    }

    // response to htlb
    io.requestor.resp.valid := io.mem.resp.valid
    io.requestor.resp.bits.addr := io.mem.resp.bits.data
    io.requestor.req.ready := true.B

    when (io.requestor.resp.valid) {
        printf("[HTW] -> [HTLB] Found HID %d to be %x\n", io.requestor.req.bits.bits.hid, io.requestor.resp.bits.addr)
    }

    switch (state) {
        is (s_ready) {
            next_state := Mux(io.requestor.req.valid, s_req, s_ready)
        }
        is (s_req) {
            when (io.mem.resp.valid) {
                next_state := s_ready
            }
        }
    }

    /*
    io.mem.s1_kill := false.B
    io.mem.s2_kill := false.B

    when (io.req.valid) {
        val hte_vaddr = io.dpath.customCSRs.htBase + io.req(w).bits.addr * 64.U

        io.mem.req.valid := true.B
        io.mem.req.bits.phys := false.B
        io.mem.req.bits.cmd  := M_XRD
        io.mem.req.bits.size := 2.U
        io.mem.req.bits.signed := false.B
        io.mem.req.bits.addr := hte_vaddr
        io.mem.req.bits.idx.foreach(_ := 0.U) // NOTE: What is this supposed to be?
        io.mem.req.bits.dprv := PRV.S.U   // PTW accesses are S-mode by definition
        io.mem.req.bits.dv := false.B

        printf("[HTW] Lookup hid 0x%x at 0x%x\n", io.req(w).bits.addr, io.mem.req.bits.addr)

        io.resp.addr := io.mem.resp.bits.data
        io.resp.miss := true.B
    } .otherwise {
        // eh on the timing correctness of this. Fine for now
        io.mem.req.valid := false.B

        io.resp.addr := io.req(w).bits.addr
    }
    */
}