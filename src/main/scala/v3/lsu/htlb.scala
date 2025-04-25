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
  val addr = UInt(maxSVAddrBits.W)
  val miss = Bool()
  val phys = Bool()
  val try_phys = Bool()
}

class TLBHTLBResp(implicit p: Parameters) extends BoomBundle()(p) {
  val hid = UInt(handleBits.W)
  val paddr = UInt(maxSVAddrBits.W)
}

case class HTLBConfig(
    nSets: Int,
    nWays: Int
)

class HTLB(cfg: HTLBConfig)(implicit p: Parameters) extends BoomModule()(p) {
  val io = IO(new Bundle {
    val req = Flipped(Vec(memWidth, Decoupled(new HTLBReq)))
    val resp = Vec(memWidth, new HTLBResp)
    val htw = new HTLBHTWIO
    val tlb = Flipped(Vec(memWidth, Valid(new TLBHTLBResp)))
    val mem = new HellaCacheIO
    val htDump = Input(UInt(maxSVAddrBits.W))
    val htInval = Input(UInt(handleBits.W))
    val htBase = Input(UInt(xLen.W))
    val kill = Input(Bool())
    val clear_htlb = Input(Bool())
    val miss_rdy = Output(Bool())
    val ht_size = Input(UInt(xLen.W))
    val htlb_enabled = Input(Bool())
    val pht_enabled = Input(Bool())
  })

  class HTLBEntryData() extends Bundle() {
    val phys = Bool()
    val addr = UInt(maxSVAddrBits.W)
    val try_phys = Bool()
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

    def inval_try_phys() : Unit = {
      val entry = getData()
      entry.try_phys := false.B
      when (entry.phys) {
        valid := false.B
      } .otherwise {
        data := entry.asUInt
      }
    }

    def set_paddr(paddr: UInt) : Unit = {
      val entry = getData()
      entry.phys := true.B
      entry.addr := paddr
      entry.try_phys := false.B
      data := entry.asUInt
    }
  }

  // Utilities
  io.htw.htlb_enabled := io.htlb_enabled
  io.htw.pht_enabled := io.pht_enabled
  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))
  val hm_enabled = widthMap(w => !io.req(w).bits.passthrough)
  val hid = widthMap(w => io.req(w).bits.haddr(xLen - 2, handleOffsetBits))
  // val hid = widthMap(w => Cat(0.U(15.W), io.req(w).bits.haddr(19, 4) % io.ht_size))
  val idxBits = log2Ceil(cfg.nSets)

  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets, Vec(cfg.nWays, new Entry(cfg.nSets))))

  // State Machine
  val s_ready :: s_request :: s_wait :: s_victim_req :: s_victim_wait :: s_dump :: s_dump_req :: s_dump_wait :: s_dumped :: Nil = Enum(9)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)
  state := Mux(io.htlb_enabled, OptimizationBarrier(next_state), s_ready)

  when (io.htBase =/= 0.U && state =/= next_state && boomParams.enableStateTracing.B) {
    midas.targetutils.SynthesizePrintf(printf("[H%d,%d\n", state, next_state))
  }

  // Refill State
  val do_refill = io.htw.resp.valid
  val hid_req = Reg(UInt(handleBits.W))

  val l1_plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")

  // Hit Logic
  val hid_tag = widthMap(w => Split(hid(w), idxBits)._1)
  val hid_set = widthMap(w => Split(hid(w), idxBits)._2)
  val htlb_hit = Wire(Vec(memWidth, Bool()))
  val htlb_miss = Wire(Vec(memWidth, Bool()))
  val real_hits = Wire(Vec(memWidth, UInt(cfg.nWays.W)))
  val hitVec = Wire(Vec(memWidth, Vec(cfg.nWays, Bool())))
  for (w <- 0 until memWidth) {
    // printf("w: %d, hid: %x, hid_tag: %x, hid_set: %x\n", w.U, hid(w), hid_tag(w), hid_set(w))
    hitVec(w) := entries(hid_set(w)).map(hm_enabled(w) && _.hit(hid_tag(w)))
    // printf("hitVec: %x\n", hitVec(w).asUInt)
    real_hits(w) := hitVec(w).asUInt
    // printf("real_hits: %x\n", real_hits(w))
    htlb_hit(w) := real_hits(w).orR
    // printf("htlb_hit: %x\n", htlb_hit(w))
    htlb_miss(w) := hm_enabled(w) && !htlb_hit(w)
    // printf("htlb_miss: %x\n", htlb_miss(w))
  }

  // Miss Logic
  val victim_entry = Reg(new EvictionReq)
  io.htw.evict.valid := state === s_victim_req
  io.htw.evict.bits := victim_entry
  io.htw.l1_dumped := state === s_dumped

  // L2 HTLB Request
  io.htw.req.valid := state === s_request
  io.htw.req.bits.valid := !io.kill
  io.htw.req.bits.bits.hid := hid_req
  
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

  for (w <- 0 until memWidth) {
    when(io.req(w).valid && hm_enabled(w)) {
      when(real_hits(w).orR) {
        // midas.targetutils.SynthesizePrintf(
        //   printf("[Hh%x,%x,%d,%d\n", hid(w), hid_tag(w), hid_set(w), OH1ToUInt(real_hits(w))
        // ))
        l1_plru.access(hid_set(w), OH1ToUInt(real_hits(w)))
      }
    }
  }

  // More Utilities
  val paddr_opt_enabled = io.htBase(xLen-1)
  val addr = widthMap(w =>
    entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData).addr)
  val phys = widthMap(w =>
    entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData).phys)
  val try_phys = widthMap(w =>
    entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData).try_phys && !entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData).phys && paddr_opt_enabled)

  // Send response to LSU
  io.miss_rdy := state === s_ready
  for (w <- 0 until memWidth) {
    // handle original case
    val sum = io.req(w).bits.haddr
    val ea_sign = Mux(sum(vaddrBits - 1),
      ~sum(63, vaddrBits) === 0.U,
      sum(63, vaddrBits) =/= 0.U
    )
    val effective_address = Cat(ea_sign, sum(vaddrBits - 1, 0)).asUInt
    val cross_pages = WireDefault(false.B)

    io.req(w).ready := true.B
    io.resp(w).miss := (do_refill && !io.req(w).bits.passthrough) || htlb_miss(w) || (htlb_hit(w) && cross_pages && phys(w))
    io.resp(w).addr := Mux(!hm_enabled(w), effective_address,
                          Mux (!io.resp(w).miss, 
                               addr(w) + io.req(w).bits.haddr(handleOffsetBits - 1, 0), 0.U))
    io.resp(w).phys := Mux(hm_enabled(w) && htlb_hit(w), phys(w), false.B)
    io.resp(w).try_phys := Mux(hm_enabled(w) && htlb_hit(w), try_phys(w) && !cross_pages, false.B)

    // midas.targetutils.PerfCounter(do_refill && io.req(0).bits.passthrough, "l1_htlb_unnecessary_miss", "L1 HTLB Unnecessary Miss")

    // you will try phys or have phys, check if you cross pages
    when (hm_enabled(w) && (htlb_hit(w) && (try_phys(w) || phys(w)))) {
      when ((addr(w) + io.req(w).bits.haddr(handleOffsetBits - 1, 0))(vaddrBits-1, pgIdxBits) =/= addr(w)(vaddrBits-1, pgIdxBits)) {
        // midas.targetutils.SynthesizePrintf(printf("[Ht%d,%x,%x\n", hid(w), addr(w), io.req(w).bits.haddr(handleOffsetBits - 1, 0)))
        // midas.targetutils.SynthesizePrintf(printf("[HTLB] object %d at vaddr %x accessed at offset %x crossed page boundaries\n", hid(w), addr(w), io.req(w).bits.haddr(handleOffsetBits - 1, 0)))
        cross_pages := true.B
        entries(hid_set(w))(OHToUInt(real_hits(w))).inval_try_phys()
      }
    }

    when(!io.resp(w).miss && io.req(w).valid && hm_enabled(w) && boomParams.enableStateTracing.B) {
      midas.targetutils.SynthesizePrintf(printf(
        "H%x,%d,%d,%x\n",
        io.resp(w).addr,
        io.resp(w).phys,
        io.resp(w).try_phys,
        io.req(w).bits.haddr,
      ))
    }
  }

  for (w <- 0 until memWidth) {
    when(io.req(w).fire && htlb_miss(w) && state === s_ready &&
         hm_enabled(w)) {
      hid_req := hid(w)
    }

    when(io.tlb(w).valid) {
      // debug print for try_phys handlen optimization
      // midas.targetutils.SynthesizePrintf(printf("[HTLB] paddr for hid %x: %x\n", io.tlb(w).bits.hid, io.tlb(w).bits.paddr))
      // midas.targetutils.SynthesizePrintf(printf("[Hp%x,%x\n", io.tlb(w).bits.hid, io.tlb(w).bits.paddr))

      val (paddr_hid_tag, paddr_hid_set) = Split(io.tlb(w).bits.hid, idxBits)
      val hitVecPAddr = entries(paddr_hid_set).map(_.hit(paddr_hid_tag))
      // midas.targetutils.SynthesizePrintf(printf("[HTLB] paddr_hid_tag: %x, paddr_hid_set: %x, way: %d, setting hid %x to paddr %x\n", paddr_hid_tag, paddr_hid_set, OHToUInt(hitVecPAddr), io.tlb(w).bits.hid, io.tlb(w).bits.paddr))
      // midas.targetutils.SynthesizePrintf(printf("[Hpi%x,%x,%d,%x,%x\n", paddr_hid_tag, paddr_hid_set, OHToUInt(hitVecPAddr), io.tlb(w).bits.hid, io.tlb(w).bits.paddr))
      entries(paddr_hid_set)(OHToUInt(hitVecPAddr)).set_paddr(io.tlb(w).bits.paddr)
    }
  }


  val have_victim = RegInit(false.B)
  // Refill L1 HTLB once L2 HTLB responds
  when(do_refill) {
    val newEntry = Wire(new HTLBEntryData)
    newEntry.phys := io.htw.resp.bits.hte.phys
    newEntry.addr := io.htw.resp.bits.hte.addr
    newEntry.try_phys := io.htw.resp.bits.hte.try_phys

    when (boomParams.enableStateTracing.B) {
      midas.targetutils.SynthesizePrintf(printf(
        "Hn%x,%d,%d\n",
        newEntry.addr,
        newEntry.phys,
        hid_req,
      ))
    }

    val (r_tag, r_idx) = Split(hid_req, idxBits)
    val candidate_repl_way = (if (cfg.nWays > 1)  l1_plru.way(r_idx) else 0.U)
    val repl_way = 
      if (cfg.nWays > 1)
            Mux(
              entries(r_idx).map(_.valid).asUInt.andR,
              candidate_repl_way,
              OHToUInt(PriorityEncoderOH(~entries(r_idx).map(_.valid).asUInt)),
            )
          else 1.U(1.W)
    // midas.targetutils.SynthesizePrintf(printf("[HTLB] Replacing way: %d, with tag: %d in set: %d\n", repl_way, r_tag, r_idx))
    val victim_line = entries(r_idx)(repl_way)
    // midas.targetutils.SynthesizePrintf(printf("[HTLB] Replacing Entry: Valid: %d, Tag: %d, Addr: %x, Phys: %d\n", victim_line.valid, victim_line.tag, victim_line.getData().addr, victim_line.getData().phys))
    // make a copy of the victim entry, and set the victim flag to notify the L2 HTLB
    have_victim := victim_line.valid

    victim_entry.addr := victim_line.data.asTypeOf(new HTLBEntryData).addr
    victim_entry.phys := victim_line.data.asTypeOf(new HTLBEntryData).phys
    victim_entry.try_phys := victim_line.data.asTypeOf(new HTLBEntryData).try_phys
    victim_entry.hid := Cat(victim_line.tag, r_idx)

    victim_line.insert(r_tag, newEntry)
  }


  // FSM Logic - get .way from plru, access it, get .way again. Do it until counter === n_ways for hits, go to final state, this marks completion, send resp to commit somehow, and then this is the end of the instruction.

  // Handle HTW Response
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
  val ways_dumped = RegInit(0.U((log2Ceil(cfg.nWays) + 1).W))
  val dumped_entry_idx = RegInit(0.U((log2Ceil(cfg.nSets * cfg.nWays) + 1).W))
  
  val hid_to_dump = RegInit(0.U(handleBits.W))
  val hit = WireDefault(false.B)

  val ways_dumped_update = (state === s_dump && next_state =/= s_dump_req) || (state === s_dump_wait && mem_resp_valid)
  val set_idx_update = ways_dumped === (cfg.nWays - 1).U && ways_dumped_update && next_state === s_dump 
  // midas.targetutils.SynthesizePrintf(printf("ways_dumped_update: %d, set_idx_update: %d\n", ways_dumped_update, set_idx_update))

  val ways_dumped_clear = set_idx_update || (state === s_dump_req && ways_dumped === (cfg.nWays + 1).U) || (state === s_dump && next_state === s_ready)
  val set_idx_clear = (state === s_dump && next_state === s_ready)
  // midas.targetutils.SynthesizePrintf(printf("ways_dumped_clear: %d, set_idx_clear: %d\n", ways_dumped_clear, set_idx_clear))

  when (set_idx < cfg.nSets.U && ways_dumped < cfg.nWays.U && state === s_dump) {
    val way = l1_plru.way(set_idx(idxBits-1,0))
    // midas.targetutils.SynthesizePrintf(printf("[HTLB] Dumping L1 Entry %d, %d\n", set_idx, way))

    hit := entries(set_idx(idxBits-1,0))(way).valid
    when(hit) {
      val entry = entries(set_idx(idxBits-1,0))(way).data.asTypeOf(new HTLBEntryData)
      // midas.targetutils.SynthesizePrintf(printf(
      //   "[HTLB] L1Entry: %d: %d,  %x, %x (%d)\n",
      //   way,
      //   entries(set_idx(idxBits-1,0))(way).valid,
      //   entries(set_idx(idxBits-1,0))(way).tag,
      //   entry.addr,
      //   entry.phys,
      //   // entry.immovable
      // ))
      hid_to_dump := Cat(entries(set_idx(idxBits-1,0))(way).tag, set_idx(idxBits - 1, 0))
      next_state := s_dump_req
    }
    l1_plru.access(set_idx(idxBits-1,0), way)
  }

  ways_dumped := Mux(ways_dumped_clear, 0.U, Mux(ways_dumped_update, ways_dumped + 1.U, ways_dumped))
  set_idx := Mux(set_idx_update, set_idx + 1.U, 
                     Mux(state === s_dump_req && ways_dumped === cfg.nWays.U, hid_to_dump(idxBits-1,0) + 1.U, 
                         Mux(set_idx_clear, 0.U, set_idx)))
  // midas.targetutils.SynthesizePrintf(printf("set_idx: %d, ways_dumped: %d\n", set_idx, ways_dumped))

  io.htw.l1miss := do_refill || htlb_miss.orR
  midas.targetutils.PerfCounter(htlb_miss.orR, "l1_htlb_miss", "l1_htlb_miss")
  when(io.htw.l1miss) {
    printf("[HTLB] L1 Miss\n")
  }

  // val hte_dst_addr = io.htDump + (dumped_entry_idx / 2.U) *4.U
  val hte_dst_addr = io.htDump + dumped_entry_idx*4.U

  io.mem.keep_clock_enabled := false.B
  io.mem.req.valid := state === s_dump_req
  io.mem.req.bits.phys := io.pht_enabled
  io.mem.req.bits.cmd := M_XWR
  io.mem.req.bits.size := 2.U // log2Ceil(xLen/8).U
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.addr := hte_dst_addr
  io.mem.req.bits.idx.foreach(_ := hte_dst_addr) // TODO: huh?
  io.mem.req.bits.dprv := PRV.U.U // HTW accesses are U-mode by definition
  io.mem.req.bits.dv := false.B
  io.mem.req.bits.tag := DontCare
  io.mem.req.bits.no_resp := false.B
  io.mem.req.bits.no_alloc := DontCare
  io.mem.req.bits.no_xcpt := DontCare
  io.mem.req.bits.data := DontCare
  io.mem.req.bits.mask := DontCare

  io.mem.s1_kill := false.B
  io.mem.s1_data.data := hid_to_dump
  io.mem.s1_data.mask := ((1 << coreDataBytes) - 1).U
  io.mem.s2_kill := false.B

  switch (state) {
    is (s_ready) {
      next_state := Mux(io.htDump.orR, s_dump, Mux(have_victim, s_victim_req, Mux(io.req(0).fire && htlb_miss(0), s_request, s_ready)))

      when (next_state === s_dump) {
        if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(printf("[HD\n"))
        }
        dumped_entry_idx := 0.U
      }
    }
    is (s_request) {
      next_state := Mux(io.htw.req.fire, s_wait, s_request)
      when(io.htw.req.fire && boomParams.enableStateTracing.B) {
        midas.targetutils.SynthesizePrintf(printf("[Hl%x\n", io.htw.req.bits.bits.hid))
      }
    }
    is (s_wait) {
      next_state := Mux(io.htw.resp.valid, s_ready, s_wait)
    }
    is (s_victim_req) {
      next_state := Mux(io.htw.evict.fire, s_victim_wait, s_victim_req)
      if (boomParams.enableStateTracing) {
        midas.targetutils.SynthesizePrintf(printf("[Hv%x,%x\n",
          victim_entry.hid,
          victim_entry.addr
        ))
      }
      when (io.htw.evict.fire) {
        have_victim := false.B
      }
    }
    is (s_victim_wait) {
      next_state := Mux(io.htw.evict_resp, s_ready, s_victim_wait)
    }
    is (s_dump_req) {
      if (boomParams.enableStateTracing) {
        midas.targetutils.SynthesizePrintf(printf("[Hd%d,%x\n", hid_to_dump, hte_dst_addr))
      }
      next_state := Mux(io.mem.req.fire, s_dump_wait, s_dump_req)
    }
    is (s_dump_wait) {
      if (boomParams.enableStateTracing) {
        midas.targetutils.SynthesizePrintf(printf("[Hd%d,%x\n", hid_to_dump, hte_dst_addr))
      }
      next_state := Mux(mem_resp_valid, s_dump, Mux(io.mem.s2_nack, s_dump_req, s_dump_wait))
      dumped_entry_idx := Mux(mem_resp_valid, dumped_entry_idx + 1.U, dumped_entry_idx)

      when (next_state === s_dump && boomParams.enableStateTracing.B) {
        midas.targetutils.SynthesizePrintf(printf("[HD%d,%d\n", dumped_entry_idx, hid_to_dump))
      }
    }
    is (s_dump) {
      next_state := Mux(dumped_entry_idx === (cfg.nSets * cfg.nWays).U, s_dumped, s_dump_req)
      when (next_state === s_dumped && boomParams.enableStateTracing.B) {
        midas.targetutils.SynthesizePrintf(printf("[HDD\n"))
      }
    }
    is (s_dumped) {
      next_state := Mux(!io.htDump.orR, s_ready, s_dumped)
    }
  }

  // Reset Logic
  when(reset.asBool || io.clear_htlb) {
    when (io.clear_htlb && boomParams.enableStateTracing.B) {
      midas.targetutils.SynthesizePrintf(printf("[Hc\n"))
    }
    entries.foreach(_.foreach(_.invalidate()))
  }

  // Invalidation Logic
  // NOTE: does this need to reset the state to s_ready? we already fence before/after the inval in the runtime.
  when (io.htInval.orR) {
    when (io.htInval === ((BigInt(1) << handleBits) - 1).U) {
      if (boomParams.enableStateTracing) {
        midas.targetutils.SynthesizePrintf(printf("[HI\n"))
      }
      entries.foreach(_.foreach(_.invalidate()))
    }.otherwise {
      if (boomParams.enableStateTracing) {
        midas.targetutils.SynthesizePrintf(printf("[Hi%x\n", io.htInval))
      }
      val (e_tag, e_idx) = Split(io.htInval, idxBits)
      for (e <- entries(e_idx)) {
        when(e_tag === e.tag) {
          e.invalidate()
        }
      }
    }
  }
}
