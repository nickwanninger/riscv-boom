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



class TimelineTracker()(implicit val p: Parameters) extends HasBoomCoreParameters {
  val cycle = if (boomParams.enableStateTracing) {
    val c = RegInit(0.U(64.W))
    c := c + 1.U
    c
  } else { 0.U }

  def mark(thread: String, event: String): Unit = {
    if (boomParams.enableStateTracing) {
      midas.targetutils.SynthesizePrintf(printf(s"TL(M,$thread,$event,%d)\n", cycle))
    }
  }

  def trackStateValue(thread: String, stateName: String, stateValue: UInt, sig: Bool): Unit = {
    if (boomParams.enableStateTracing) {
      val startCycle  = RegInit(0.U(64.W))
      val valueAtStart = RegInit(0.U(stateValue.getWidth.W))
      val prev        = RegNext(sig, false.B)
      when(sig && !prev) {
        startCycle   := cycle
        valueAtStart := stateValue
      }
      when(!sig && prev) {
        midas.targetutils.SynthesizePrintf(
          printf(s"TL(B,$thread,$stateName %x,%d,%d,0)\n", valueAtStart, cycle, startCycle)
        )
      }
    }
  }

  def trackState(thread: String, event: String, sig: Bool, value: UInt = 0.U): Unit = {
    if (boomParams.enableStateTracing) {
      val startCycle = RegInit(0.U(64.W))
      val prev       = RegNext(sig, false.B)
      when(sig && !prev) { startCycle := cycle }
      when(!sig && prev) {
        midas.targetutils.SynthesizePrintf(
          printf(s"TL(B,$thread,$event,%d,%d,%d)\n", cycle, startCycle, value)
        )
      }
    }
  }

  def trackCounter(thread: String, count: UInt): Unit = {
    if (boomParams.enableStateTracing) {
      val prev = RegNext(count, 0.U)
      when(count =/= prev) {
        midas.targetutils.SynthesizePrintf(printf(s"TL(C,$thread,%d,%d)\n", cycle, count))
      }
    }
  }
}





class HTLBSimple(cfg: HTLBConfig)(implicit p: Parameters)
    extends BoomModule()(p) {
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
    val htInvald = Output(Bool())

    // XXX: refilling was used to detect nonsense misses in the LSU, may cause
    //      combinational loops in FireSim's token model. Disabled for now.
    // val refilling = Valid(UInt(handleBits.W))
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

    def inval_try_phys(): Unit = {
      val entry = getData()
      entry.try_phys := false.B
      when(entry.phys) {
        valid := false.B
      }.otherwise {
        data := entry.asUInt
      }
    }

    def set_paddr(paddr: UInt): Unit = {
      val entry = getData()
      entry.phys := true.B
      entry.addr := paddr
      entry.try_phys := false.B
      data := entry.asUInt
    }
  }

  val twoStageHTW = boomParams.enableTwoStageHTW

  val timeline = new TimelineTracker()

  // Utilities
  val hm_enabled = !io.req(0).bits.passthrough
  val hid = io.req(0).bits.haddr(xLen - 2, handleOffsetBits)
  val idxBits = log2Ceil(cfg.nSets)

  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets, Vec(cfg.nWays, new Entry(cfg.nSets))))

  val l1_plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")

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

  // The requested HID
  val hid_req = Reg(UInt(handleBits.W))

  // State Machine
  // - ready: idle, waiting for a miss.
  // - request_ht_directory: read cache, or issue a request for the HT directory entry
  // - wait_ht_directory: wait for the HT directory entry to return
  // - request_ht_entry: read cache, or issue a request for the HT entry
  // - wait_ht_entry: wait for the HT entry to return. Refill when done.
  val s_ready :: s_request_ht_directory :: s_wait_ht_directory :: s_request_ht_entry :: s_wait_ht_entry :: Nil = Enum(5)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)


  // Consider the number of times the HTLB is not ready (that is, it is dealing with a miss, blocking the LSU)
  midas.targetutils.PerfCounter(state =/= s_ready, "htlb_waiting", "htlb_waiting")
  midas.targetutils.PerfCounter(io.req(0).fire && htlb_miss, "htlb_miss", "htlb_miss")
  midas.targetutils.PerfCounter(io.req(0).fire && htlb_hit && hm_enabled, "htlb_hit", "htlb_hit")
  midas.targetutils.PerfCounter(state === s_request_ht_directory || state === s_request_ht_entry, "htlb_mem_stall_cycles", "htlb_mem_stall_cycles")
  midas.targetutils.PerfCounter(io.req(0).fire && htlb_miss && l0_hit_in_s_ready, "htlb_l0_hit", "htlb_l0_hit")



  // XXX: must use RegNext here to break the combinational path
  //      io.mem.resp.valid → io.mem.req.valid that exists in s_wait_ht_directory.
  //      Without this register, MIDAS's token model deadlocks at cycle 3.
  //      The 1-cycle delay is acceptable given the multi-cycle cache latency.
  val mem_resp_valid = RegNext(io.mem.resp.valid, false.B)
  val mem_resp_data  = RegNext(io.mem.resp.bits.data, 0.U)

  // HTW: L0 Cache (Top Level Cache) Setup
  // Adapted from HTW.scala
  val entries_per_ht_bits = 18
  // We assume twoStageHTW is true for this simplified version with 2-stage
  val ht_directory_cache_size = boomParams.HTWCacheSize
  def getIndex(hid: UInt) =
    (hid >> entries_per_ht_bits)(log2Ceil(ht_directory_cache_size) - 1, 0)
  def getTag(hid: UInt) = (hid >> entries_per_ht_bits)(
    handleBits - 19,
    log2Ceil(ht_directory_cache_size)
  )
  val ht_directory_cache =
    if (twoStageHTW)
      Some(
        RegInit(
          VecInit(
            Seq.fill(ht_directory_cache_size)(0.U.asTypeOf(new TopLevelCacheEntry))
          )
        )
      )
    else None

  val l0_lookup_idx = getIndex(hid_req)
  val l0_lookup_tag = getTag(hid_req)

  // Hoisted wire so htlb_l0_hit PerfCounter can reference it outside the s_ready block
  val l0_hit_in_s_ready = WireDefault(false.B)

  val inner_walk_base = RegInit(0.U(xLen.W))

  // htInval is a CSR that software writes to request an invalidation.
  // The CSR clears itself when hrInvald asserts, so we don't need pulse logic.
  val do_htInval = state === s_ready && io.htInval.orR

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
  // Uncached response handling (if ever valid)
  io.mem.uncached_resp.foreach(_.ready := true.B)

  // FSM Implementation
  state := Mux(io.htlb_enabled, next_state, s_ready)
  when(!io.htlb_enabled && state =/= s_ready) {
    midas.targetutils.SynthesizePrintf(
      printf("HTLB.disabled: cycle=%d state=%d flushed to s_ready\n",
        timeline.cycle, state)
    )
  }

  timeline.trackState("htlb.state", "request_ht_directory", state === s_request_ht_directory)
  timeline.trackState("htlb.state", "wait_ht_directory", state === s_wait_ht_directory)
  timeline.trackState("htlb.state", "request_ht_entry", state === s_request_ht_entry)
  timeline.trackState("htlb.state", "wait_ht_entry", state === s_wait_ht_entry)

  timeline.trackState("htlb.state", "mem_req", io.mem.req.valid)
  timeline.trackState("htlb.state", "mem_resp", io.mem.resp.valid)
  timeline.trackState("htlb.state", "retry_nack", io.mem.s2_nack)

  for (s <- 0 until cfg.nSets) {
    for (w <- 0 until cfg.nWays) {
      val e = entries(s)(w)
      timeline.trackState(s"htlb.s${s}w${w}", "invalid", !e.valid, e.tag)
    }
  }


  switch(state) {



    is(s_ready) {
      // Prioritize invalidation over handling a miss
      when(do_htInval) {
      } .elsewhen(io.req(0).fire && htlb_miss) {
        hid_req := hid

        // if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.miss: cycle=%d hid=0x%x htBase=0x%x\n",
              timeline.cycle, hid, io.htBase)
          )
        // }

        if (twoStageHTW) {
          val cache = ht_directory_cache.get
          val l0_idx = getIndex(hid)
          val l0_tag = getTag(hid)
          val l0_entry = cache(l0_idx)
          val l0_hit = l0_entry.valid && l0_entry.tag === l0_tag

          when(l0_hit) {
            l0_hit_in_s_ready := true.B
            inner_walk_base := l0_entry.data
            walk_addr := l0_entry.data + (hid(entries_per_ht_bits - 1, 0)) * 8.U
            io.mem.req.valid := true.B
            when(io.mem.req.fire) {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x (L0 hit)\n",
                  timeline.cycle, hid, walk_addr)
              )
              next_state := s_wait_ht_entry
            }.otherwise {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.stall: cycle=%d hid=0x%x mem busy, L0 hit → s_request_ht_entry\n",
                  timeline.cycle, hid)
              )
              next_state := s_request_ht_entry
            }
          }.otherwise {
            walk_addr := io.htBase + (hid >> entries_per_ht_bits) * 8.U
            io.mem.req.valid := true.B
            when(io.mem.req.fire) {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.walk.dir: cycle=%d hid=0x%x addr=0x%x\n",
                  timeline.cycle, hid, walk_addr)
              )
              next_state := s_wait_ht_directory
            }.otherwise {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.stall: cycle=%d hid=0x%x mem busy, L0 miss → s_request_ht_directory\n",
                  timeline.cycle, hid)
              )
              next_state := s_request_ht_directory
            }
          }
        } else {
          walk_addr := io.htBase + hid * 8.U
          io.mem.req.valid := true.B
          when(io.mem.req.fire) {
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x\n",
                timeline.cycle, hid, walk_addr)
            )
            next_state := s_wait_ht_entry
          }.otherwise {
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.stall: cycle=%d hid=0x%x mem busy → s_request_ht_entry\n",
                timeline.cycle, hid)
            )
            next_state := s_request_ht_entry
          }
        }
      }
    }


    is(s_request_ht_directory) {
      if (twoStageHTW) {
        // Request L0 entry from memory
        // Addr = htBase + (hid >> 18) * 8
        walk_addr := io.htBase + (hid_req >> entries_per_ht_bits) * 8.U
        io.mem.req.valid := true.B

        // Only advance state if the request actually fired
        when(io.mem.req.fire) {
          // if (boomParams.enableStateTracing) {
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.walk.dir: cycle=%d hid=0x%x addr=0x%x\n",
                timeline.cycle, hid_req, walk_addr)
            )
          // }
          next_state := s_wait_ht_directory
        }
      }
    }



    is(s_wait_ht_directory) {
      if (twoStageHTW) {
        // Wait for outer walk response
        when(mem_resp_valid) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.dir_resp: cycle=%d hid=0x%x base=0x%x\n",
              timeline.cycle, hid_req, mem_resp_data)
          )

          // Update L0 Cache
          val cache = ht_directory_cache.get
          val fill_idx = getIndex(hid_req)
          cache(fill_idx).valid := true.B
          cache(fill_idx).tag := getTag(hid_req)
          cache(fill_idx).data := mem_resp_data

          inner_walk_base := mem_resp_data

          // Fast Path: Request Second Level immediately
          // Addr = mem_resp_data + (hid_req & mask) * 8
          walk_addr := mem_resp_data + (hid_req(entries_per_ht_bits - 1, 0)) * 8.U
          io.mem.req.valid := true.B

          when (io.mem.req.fire) {
            midas.targetutils.PerfCounter(mem_resp_valid && io.mem.req.fire, "htlb_fast_path_fired", "htlb_fast_path_fired")
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x\n",
                timeline.cycle, hid_req, walk_addr)
            )
            next_state := s_wait_ht_entry
          } .otherwise {
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.stall: cycle=%d hid=0x%x mem busy after dir_resp → s_request_ht_entry\n",
                timeline.cycle, hid_req)
            )
            next_state := s_request_ht_entry
          }

        }.elsewhen(io.mem.s2_nack) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.nack: cycle=%d hid=0x%x in wait_ht_directory → s_request_ht_directory\n",
              timeline.cycle, hid_req)
          )
          next_state := s_request_ht_directory
        }.elsewhen(io.mem.s2_xcpt.asUInt.orR) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.xcpt: cycle=%d hid=0x%x in wait_ht_directory → s_request_ht_directory\n",
              timeline.cycle, hid_req)
          )
          next_state := s_request_ht_directory
        }
      }
    }



    is(s_request_ht_entry) {
      // Request Inner Entry (HTE)
      // Addr = inner_walk_base + (hid & mask) * 8
      // mask for 18 bits = (1 << 18) - 1
      if (twoStageHTW) {
        walk_addr := inner_walk_base + (hid_req(
          entries_per_ht_bits - 1,
          0
        )) * 8.U
      } else {
        walk_addr := io.htBase + hid_req * 8.U
      }

      io.mem.req.valid := true.B

      when(io.mem.req.fire) {
        midas.targetutils.SynthesizePrintf(
          printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x\n",
            timeline.cycle, hid_req, walk_addr)
        )
        next_state := s_wait_ht_entry
      }
    }



    is(s_wait_ht_entry) {
      when(mem_resp_valid) {
        // Insert into L1 entries at valid hid_req
        // No victim handling needed for simplificaiton, just overwrite
        val newEntry = Wire(new HTLBEntryData)
        val hte = mem_resp_data.asTypeOf(new HTE)
        newEntry.phys := hte.phys
        newEntry.addr := hte.addr
        newEntry.try_phys := hte.try_phys

        val (r_tag, r_idx) = Split(hid_req, idxBits)
        // Use PLRU to pick way or first invalid
        val candidate_repl_way =
          (if (cfg.nWays > 1) l1_plru.way(r_idx) else 0.U)
        val repl_way =
          if (cfg.nWays > 1)
            Mux(
              entries(r_idx).map(_.valid).asUInt.andR,
              candidate_repl_way,
              OHToUInt(PriorityEncoderOH(~entries(r_idx).map(_.valid).asUInt))
            )
          else 0.U

        entries(r_idx)(repl_way).insert(r_tag, newEntry)

        midas.targetutils.SynthesizePrintf(
          printf("HTLB.fill: cycle=%d hid=0x%x addr=0x%x phys=%d try_phys=%d → s_ready\n",
            timeline.cycle, hid_req, newEntry.addr, newEntry.phys, newEntry.try_phys)
        )
        next_state := s_ready

      }.elsewhen(io.mem.s2_nack) {
        midas.targetutils.SynthesizePrintf(
          printf("HTLB.nack: cycle=%d hid=0x%x in wait_ht_entry → s_request_ht_entry\n",
            timeline.cycle, hid_req)
        )
        next_state := s_request_ht_entry
      }.elsewhen(io.mem.s2_xcpt.asUInt.orR) {
        midas.targetutils.SynthesizePrintf(
          printf("HTLB.xcpt: cycle=%d hid=0x%x in wait_ht_entry → s_request_ht_entry\n",
            timeline.cycle, hid_req)
        )
        next_state := s_request_ht_entry
      }
    }



  }

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Response & Output Logic
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  val paddr_opt_enabled = boomParams.enableHTLBPhysAddr.B

  val entry_data =
    entries(hid_set)(OHToUInt(real_hits)).data.asTypeOf(new HTLBEntryData)
  val addr = entry_data.addr
  val phys = entry_data.phys
  val try_phys = entry_data.try_phys && !entry_data.phys && paddr_opt_enabled

  val addr_crossed_pages =
    (addr + io.req(0).bits.haddr(handleOffsetBits - 1, 0))(
      vaddrBits - 1,
      pgIdxBits
    ) =/= addr(vaddrBits - 1, pgIdxBits)

  io.miss_rdy := state === s_ready

  val sum = io.req(0).bits.haddr
  val ea_sign = Mux(
    sum(vaddrBits - 1),
    ~sum(63, vaddrBits) === 0.U,
    sum(63, vaddrBits) =/= 0.U
  )
  val effective_address = Cat(ea_sign, sum(vaddrBits - 1, 0)).asUInt
  val cross_pages = WireDefault(false.B)

  io.req(0).ready := true.B
  io.resp(0).miss := htlb_miss || (htlb_hit && cross_pages && phys)

  io.resp(0).addr := Mux(
    !hm_enabled,
    effective_address,
    Mux(
      !io.resp(0).miss,
      addr + io.req(0).bits.haddr(handleOffsetBits - 1, 0),
      0.U
    )
  )
  io.resp(0).phys := Mux(hm_enabled && htlb_hit, phys, false.B)
  io.resp(0).try_phys := Mux(
    hm_enabled && htlb_hit,
    try_phys && !cross_pages,
    false.B
  )

  // Cross page logic
  val could_return_phys = hm_enabled && (htlb_hit && (try_phys || phys))
  cross_pages := could_return_phys && addr_crossed_pages
  when(could_return_phys && addr_crossed_pages) {
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
    entries(paddr_hid_set)(OHToUInt(hitVecPAddr)).set_paddr(
      io.tlb(0).bits.paddr
    )
  }

  // Reset
  when(reset.asBool || io.clear_htlb) {
    entries.foreach(_.foreach(_.invalidate()))
    ht_directory_cache.foreach(_.foreach(_.valid := false.B))
    when(io.clear_htlb) {
      midas.targetutils.SynthesizePrintf(
        printf("HTLB.clear_htlb: cycle=%d all entries invalidated\n", timeline.cycle)
      )
    }
  }

  // Invalidation
  when(do_htInval) {
    midas.targetutils.SynthesizePrintf(
      printf("HTLB.htInval: cycle=%d hid=0x%x\n", timeline.cycle, io.htInval)
    )
    // Invalidate L1
    when(io.htInval === ((BigInt(1) << handleBits) - 1).U) {
      entries.foreach(_.foreach(_.invalidate()))
      ht_directory_cache.foreach(_.foreach(_.valid := false.B))
    }.otherwise {
      val (e_tag, e_idx) = Split(io.htInval, idxBits)
      for (e <- entries(e_idx)) {
        when(e_tag === e.tag) {
          e.invalidate()
        }
      }

      // if (twoStageHTW) {
      //   val l0_idx = getIndex(io.htInval)
      //   val l0_tag = getTag(io.htInval)
      //   val cache = ht_directory_cache.get
      //   when(cache(l0_idx).valid && cache(l0_idx).tag === l0_tag) {
      //     cache(l0_idx).valid := false.B
      //   }
      // }

    }
  }

  io.htInvald := do_htInval
}
