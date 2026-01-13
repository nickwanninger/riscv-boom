package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.constants._

import boom.v3.common._
import freechips.rocketchip.tile.CoreBundle
import freechips.rocketchip.jtag.JtagState.State.width
import freechips.rocketchip.tilelink.TLMessages.d


class HTLBSimple(cfg: HTLBConfig)(implicit p: Parameters) extends BoomModule()(p) {
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

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Data Structures
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

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

  class HandleTraceQueue(width: Int, log2Depth: Int)(implicit p: Parameters) extends BoomModule()(p) {
    val io = IO(new Bundle {
      val enq = Flipped(Decoupled(UInt(width.W)))
    })

    val depth = 1 << log2Depth
    val queue = Reg(Vec(depth, UInt(width.W)))
    val head = RegInit(0.U(log2Depth.W))
    val tail = RegInit(0.U(log2Depth.W))
    val count = RegInit(0.U((log2Depth + 1).W))

    val full = count === depth.U
    io.enq.ready := true.B

    when(io.enq.valid) {
      queue(tail) := io.enq.bits
      tail := tail + 1.U
      
      when (full) {
        head := head + 1.U
      } .otherwise {
        count := count + 1.U
      }
      
      if (boomParams.enableStateTracing) {
         val start_offset = Mux(full, 1.U, 0.U)
         val num_old_items = Mux(full, (depth - 1).U, count)
         
         // Using simpler masking for circular access
         val mask = (depth - 1).U

         midas.targetutils.SynthesizePrintf(printf("[H Queue] %d/%d: ", head, tail))
         
         for (i <- 0 until depth) {
             // Access with mask to handle wrap
             val idx = (head + start_offset + i.U) & mask

             when (i.U < num_old_items) {
                 midas.targetutils.SynthesizePrintf(printf("%x ", queue(idx)))
             } .elsewhen (i.U === num_old_items) {
                 midas.targetutils.SynthesizePrintf(printf("%x ", io.enq.bits))
             }
         }
         midas.targetutils.SynthesizePrintf(printf("\n"))
      }
    }
  }

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // L1 HTLB Logic
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  // Utilities
  val hm_enabled = !io.req(0).bits.passthrough
  val hid = io.req(0).bits.haddr(xLen - 2, handleOffsetBits)
  val idxBits = log2Ceil(cfg.nSets)

  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets, Vec(cfg.nWays, new Entry(cfg.nSets))))

  val l1_plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")
  val trace_queue = Module(new HandleTraceQueue(handleBits, 4))
  trace_queue.io.enq.valid := false.B
  trace_queue.io.enq.bits := 0.U

  // Hit Logic
  val hid_tag = Split(hid, idxBits)._1
  val hid_set = Split(hid, idxBits)._2
  val htlb_hit = Wire(Bool()) 
  val htlb_miss = Wire(Bool()) 
  val real_hits = Wire(UInt(cfg.nWays.W)) 
  val hitVec = Wire(Vec(cfg.nWays, Bool())) 
  
  hitVec := entries(hid_set).map(hm_enabled && _.hit(hid_tag))
  real_hits := hitVec.asUInt
  htlb_hit := real_hits.orR
  htlb_miss := hm_enabled && !htlb_hit

   // Update PLRU
  when(io.req(0).valid && hm_enabled) {
    when(real_hits.orR) {
      l1_plru.access(hid_set, OH1ToUInt(real_hits))
    }
  }

  // Refill Logic Setup
  val hid_req = Reg(UInt(handleBits.W))

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // State Machine (HTW Internalized)
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  val s_ready :: s_req_l0 :: s_check_l0 :: s_req_outer :: s_wait_outer :: s_req_inner :: s_wait_inner :: Nil = Enum(7)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)

  val mem_resp_valid = RegNext(io.mem.resp.valid)
  val mem_resp_data = RegNext(io.mem.resp.bits.data)
  
  // Uncached response handling (if ever valid)
  io.mem.uncached_resp.map { resp =>
    assert(!(resp.valid && io.mem.resp.valid))
    resp.ready := true.B
    when(resp.valid) {
      mem_resp_valid := true.B
      mem_resp_data := resp.bits.data
    }
  }

  // HTW: L0 Cache (Top Level Cache) Setup
  // Adapted from HTW.scala
  val entries_per_ht_bits = 18
  // We assume boomParams.enableTwoStageHTW is true for this simplified version with 2-stage
  val top_level_cache_size = boomParams.HTWCacheSize
  def getIndex(hid: UInt) = (hid >> entries_per_ht_bits)(log2Ceil(top_level_cache_size)-1, 0)
  def getTag(hid: UInt) = (hid >> entries_per_ht_bits)(handleBits-19, log2Ceil(top_level_cache_size))
  val top_level_cache = if (boomParams.enableTwoStageHTW) Some(RegInit(VecInit(Seq.fill(top_level_cache_size)(0.U.asTypeOf(new TopLevelCacheEntry))))) else None
  
  val l0_lookup_idx = getIndex(hid_req)
  val l0_lookup_tag = getTag(hid_req)
  
  val inner_walk_base = RegInit(0.U(xLen.W))

  // Memory Request Construction
  val walk_addr = Wire(UInt(xLen.W))
  walk_addr := 0.U // assignment in switch

  // IO assignments for walker
  io.mem.req.valid := false.B
  io.mem.req.bits.phys := io.pht_enabled
  io.mem.req.bits.cmd := M_XRD
  io.mem.req.bits.size := log2Ceil(xLen / 8).U
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.addr := walk_addr
  io.mem.req.bits.idx.foreach(_ := walk_addr)
  io.mem.req.bits.dprv := Mux(io.pht_enabled, PRV.S.U, PRV.U.U)
  io.mem.req.bits.dv := false.B
  io.mem.req.bits.tag := DontCare
  io.mem.req.bits.no_resp := false.B
  io.mem.req.bits.no_alloc := DontCare
  io.mem.req.bits.no_xcpt := DontCare
  io.mem.req.bits.data := DontCare
  io.mem.req.bits.mask := DontCare
  io.mem.s1_kill := false.B
  io.mem.s1_data.data := 0.U
  io.mem.s1_data.mask := 0.U
  io.mem.s2_kill := false.B
  io.mem.keep_clock_enabled := false.B


  // FSM Implementation
  state := Mux(io.htlb_enabled, next_state, s_ready)

  switch (state) {
    is (s_ready) {
       // If a simple miss, start handling it
       // Removed memWidth loop, assume w=0
       when (io.req(0).fire && htlb_miss) {
         hid_req := hid
         // Optimization: check L0 immediately? 
         if (boomParams.enableTwoStageHTW) {
             next_state := s_check_l0
         } else {
             next_state := s_req_inner
         }
       }
    }
    is (s_check_l0) {
      if (boomParams.enableTwoStageHTW) {
          // Check L0 cache
          val cache = top_level_cache.get
          val l0_entry = cache(l0_lookup_idx)
          val l0_hit = l0_entry.valid && l0_entry.tag === l0_lookup_tag

          if (boomParams.enableStateTracing) {
              midas.targetutils.SynthesizePrintf(printf("[H%d|CheckL0] HIDReq: %x, L0Hit: %b\n", state, hid_req, l0_hit))
          }

          when (l0_hit) {
            inner_walk_base := l0_entry.data
            next_state := s_req_inner
          } .otherwise {
            next_state := s_req_outer
          }
      }
    }
    is (s_req_outer) {
      if (boomParams.enableTwoStageHTW) {
          // Request L0 entry from memory
          // Addr = htBase + (hid >> 18) * 8
          walk_addr := io.htBase + (hid_req >> entries_per_ht_bits) * 8.U
          io.mem.req.valid := true.B
          
          if (boomParams.enableStateTracing) {
              midas.targetutils.SynthesizePrintf(printf("[H%d|ReqOuter] Addr: %x, Base: %x, HID: %x\n", state, walk_addr, io.htBase, hid_req))
          }

          when (io.mem.req.fire) {
            next_state := s_wait_outer
          }
      }
    }
    is (s_wait_outer) {
       if (boomParams.enableTwoStageHTW) {
           // Wait for outer walk response
           when (mem_resp_valid) {
             // Update L0 Cache
             val cache = top_level_cache.get
             val fill_idx = getIndex(hid_req)
             cache(fill_idx).valid := true.B
             cache(fill_idx).tag := getTag(hid_req)
             cache(fill_idx).data := mem_resp_data
             
             inner_walk_base := mem_resp_data
             next_state := s_req_inner
             
             if (boomParams.enableStateTracing) {
                midas.targetutils.SynthesizePrintf(printf("[H%d|RespOuter] Data: %x\n", state, mem_resp_data))
             }
           } .elsewhen(io.mem.s2_nack) {
             next_state := s_req_outer
           }
       }
    }
    is (s_req_inner) {
      // Request Inner Entry (HTE)
      // Addr = inner_walk_base + (hid & mask) * 8
      // mask for 18 bits = (1 << 18) - 1
      if (boomParams.enableTwoStageHTW) {
          walk_addr := inner_walk_base + (hid_req(entries_per_ht_bits-1, 0)) * 8.U
      } else {
          walk_addr := io.htBase + hid_req * 8.U
      }
      
      io.mem.req.valid := true.B

      if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(printf("[H%d|ReqInner] Addr: %x, Base: %x, HID: %x\n", state, walk_addr, inner_walk_base, hid_req))
      }

      when (io.mem.req.fire) {
        next_state := s_wait_inner
      }
    }
    is (s_wait_inner) {
       when (mem_resp_valid) {
         if (boomParams.enableStateTracing) {
            midas.targetutils.SynthesizePrintf(printf("[H%d|RespInner] Data: %x\n", state, mem_resp_data))
         }

          // Insert into L1 entries at valid hid_req
          // No victim handling needed for simplificaiton, just overwrite
          val newEntry = Wire(new HTLBEntryData)
          val hte = mem_resp_data.asTypeOf(new HTE)
          newEntry.phys := hte.phys
          newEntry.addr := hte.addr
          newEntry.try_phys := hte.try_phys

          val (r_tag, r_idx) = Split(hid_req, idxBits)
          // Use PLRU to pick way or first invalid
          val candidate_repl_way = (if (cfg.nWays > 1) l1_plru.way(r_idx) else 0.U)
          val repl_way = if (cfg.nWays > 1) Mux(entries(r_idx).map(_.valid).asUInt.andR, candidate_repl_way, OHToUInt(PriorityEncoderOH(~entries(r_idx).map(_.valid).asUInt))) else 0.U
          
          entries(r_idx)(repl_way).insert(r_tag, newEntry)
          
          if (boomParams.enableStateTracing) {
              midas.targetutils.SynthesizePrintf(printf("[H%d|Refill] HID: %x, Addr: %x, Phys: %b, Raw: %x\n", state, hid_req, newEntry.addr, newEntry.phys, mem_resp_data))
          }
          
          trace_queue.io.enq.valid := true.B
          trace_queue.io.enq.bits := hid_req
          
          next_state := s_ready

       } .elsewhen(io.mem.s2_nack) {
         next_state := s_req_inner
       }
    }
  }


  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Response & Output Logic
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  val paddr_opt_enabled = boomParams.enableHTLBPhysAddr.B
  
  val entry_data = entries(hid_set)(OHToUInt(real_hits)).data.asTypeOf(new HTLBEntryData)
  val addr = entry_data.addr
  val phys = entry_data.phys
  val try_phys = entry_data.try_phys && !entry_data.phys && paddr_opt_enabled
  
  val addr_crossed_pages = (addr + io.req(0).bits.haddr(handleOffsetBits - 1, 0))(vaddrBits-1, pgIdxBits) =/= addr(vaddrBits-1, pgIdxBits)

  io.miss_rdy := state === s_ready

  val sum = io.req(0).bits.haddr
  val ea_sign = Mux(sum(vaddrBits - 1), ~sum(63, vaddrBits) === 0.U, sum(63, vaddrBits) =/= 0.U)
  val effective_address = Cat(ea_sign, sum(vaddrBits - 1, 0)).asUInt
  val cross_pages = WireDefault(false.B)

  io.req(0).ready := true.B
  io.resp(0).miss := htlb_miss || (htlb_hit && cross_pages && phys)
  
  io.resp(0).addr := Mux(!hm_enabled, effective_address,
                        Mux (!io.resp(0).miss, 
                              addr + io.req(0).bits.haddr(handleOffsetBits - 1, 0), 0.U))
  io.resp(0).phys := Mux(hm_enabled && htlb_hit, phys, false.B)
  io.resp(0).try_phys := Mux(hm_enabled && htlb_hit, try_phys && !cross_pages, false.B)

  // Cross page logic
  val could_return_phys = hm_enabled && (htlb_hit && (try_phys || phys))
  cross_pages := could_return_phys && addr_crossed_pages
  when (could_return_phys && addr_crossed_pages) {
    entries(hid_set)(OHToUInt(real_hits)).inval_try_phys()
  }


  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Tie-offs and unused IO
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  io.htw.req.valid := false.B
  io.htw.req.bits.valid := false.B
  io.htw.req.bits.bits.hid := 0.U
  io.htw.evict.valid := false.B
  io.htw.evict.bits := DontCare
  io.htw.l1_dumped := false.B
  io.htw.l1miss := false.B // Can technically be hooked up if profiling needs it
  
  // Pass through enable signals
  io.htw.htlb_enabled := io.htlb_enabled
  io.htw.pht_enabled := io.pht_enabled
  
  // Tie off TLB updates
  when(io.tlb(0).valid) {
    val (paddr_hid_tag, paddr_hid_set) = Split(io.tlb(0).bits.hid, idxBits)
    val hitVecPAddr = entries(paddr_hid_set).map(_.hit(paddr_hid_tag))
    if (boomParams.enableStateTracing) {
        midas.targetutils.SynthesizePrintf(printf("Ht:%d,%x\n", io.tlb(0).bits.hid, io.tlb(0).bits.paddr))
    }
    entries(paddr_hid_set)(OHToUInt(hitVecPAddr)).set_paddr(io.tlb(0).bits.paddr)
  }

  // Reset
  when(reset.asBool || io.clear_htlb) {
    entries.foreach(_.foreach(_.invalidate()))
    top_level_cache.foreach(_.foreach(_.valid := false.B))
  }
  
  // Invalidation
  when (io.htInval.orR) {
    // Invalidate L1
    when (io.htInval === ((BigInt(1) << handleBits) - 1).U) {
      entries.foreach(_.foreach(_.invalidate()))
      top_level_cache.foreach(_.foreach(_.valid := false.B))
  }.otherwise {
      val (e_tag, e_idx) = Split(io.htInval, idxBits)
      for (e <- entries(e_idx)) {
        when(e_tag === e.tag) {
          e.invalidate()
        }
      }
      
      if (boomParams.enableTwoStageHTW) {
          val l0_idx = getIndex(io.htInval)
          val l0_tag = getTag(io.htInval)
          val cache = top_level_cache.get
          when (cache(l0_idx).valid && cache(l0_idx).tag === l0_tag) {
            cache(l0_idx).valid := false.B
          }
      }
      
    }
  }

}
