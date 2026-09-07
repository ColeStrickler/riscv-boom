package boom.v3.lsu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import boom.v3.common._
import midas.targetutils.SynthesizePrintf

/** Opt-in verification hardware, only enabled in the DM experiment target. */
case object DMVerificationKey extends Field[Boolean](false)

/** Exercise the actual DTLB with controlled PTW timing, independently of the core. */
class DMTLBRaceVerification(implicit edge: TLEdgeOut, p: Parameters)
    extends BoomModule()(p) with HasL1HellaCacheParameters {
  val io = IO(new Bundle { val done = Output(Bool()) })
  val dut = Module(new NBDTLB(false, log2Ceil(coreDataBytes),
    TLBConfig(dcacheParams.nTLBSets, dcacheParams.nTLBWays)))
  val clear :: request :: walk :: fence :: late :: probe :: newWalk :: refill :: hit :: done :: Nil = Enum(10)
  val state = RegInit(clear)
  val test = RegInit(0.U(2.W))
  val delay = RegInit(0.U(2.W))
  val cycles = RegInit(0.U(10.W))
  val oldDM = test(0)
  val delayed = test(1)
  val va = "h47000000".U(vaddrBitsExtended.W)
  val pa = "h80000000".U(paddrBits.W)
  io.done := state === done
  when (!io.done) { cycles := cycles + 1.U }
  assert(io.done || cycles < 512.U, "[dm-test] DTLB race test timed out")

  dut.io.kill := false.B
  dut.io.sfence := 0.U.asTypeOf(dut.io.sfence)
  dut.io.ptw.req.ready := true.B
  dut.io.ptw.resp := 0.U.asTypeOf(dut.io.ptw.resp)
  dut.io.ptw.ptbr := 0.U.asTypeOf(dut.io.ptw.ptbr)
  dut.io.ptw.ptbr.mode := 9.U
  dut.io.ptw.hgatp := 0.U.asTypeOf(dut.io.ptw.hgatp)
  dut.io.ptw.vsatp := 0.U.asTypeOf(dut.io.ptw.vsatp)
  dut.io.ptw.status := 0.U.asTypeOf(dut.io.ptw.status)
  dut.io.ptw.status.prv := PRV.S.U
  dut.io.ptw.status.dprv := PRV.S.U
  dut.io.ptw.hstatus := 0.U.asTypeOf(dut.io.ptw.hstatus)
  dut.io.ptw.gstatus := 0.U.asTypeOf(dut.io.ptw.gstatus)
  dut.io.ptw.customCSRs.csrs.foreach { csr =>
    csr.ren := false.B
    csr.wen := false.B
    csr.wdata := 0.U
    csr.value := 0.U
  }
  dut.io.ptw.pmp := 0.U.asTypeOf(dut.io.ptw.pmp)
  dut.io.ptw.pmp.head.cfg.a := 3.U
  dut.io.ptw.pmp.head.cfg.r := true.B
  dut.io.ptw.pmp.head.cfg.w := true.B
  dut.io.ptw.pmp.head.cfg.x := true.B
  dut.io.ptw.pmp.head.addr := Fill(dut.io.ptw.pmp.head.addr.getWidth, 1.U)
  dut.io.ptw.pmp.head.mask := Fill(paddrBits, 1.U)
  for (w <- 0 until memWidth) {
    dut.io.req(w).valid := false.B
    dut.io.req(w).bits := 0.U.asTypeOf(dut.io.req(w).bits)
    dut.io.req(w).bits.vaddr := va
    dut.io.req(w).bits.size := log2Ceil(coreDataBytes).U
    dut.io.req(w).bits.cmd := M_XRD
    dut.io.req(w).bits.prv := PRV.S.U
  }
  val response = dut.io.ptw.resp.bits
  response.level := (pgLevels - 1).U
  response.homogeneous := true.B
  response.pte.ppn := pa >> 12
  response.pte.v := true.B
  response.pte.r := true.B
  response.pte.w := true.B
  response.pte.a := true.B
  response.pte.d := true.B
  response.pte.reserved_for_software := oldDM.asUInt

  switch (state) {
    is (clear) {
      dut.io.sfence.valid := true.B
      state := request
    }
    is (request) {
      dut.io.req(0).valid := true.B
      when (dut.io.req(0).fire) {
        assert(dut.io.resp(0).miss, "[dm-test] initial translation should miss")
        state := walk
      }
    }
    is (walk) {
      when (dut.io.ptw.req.fire) {
        assert(dut.io.ptw.req.bits.valid)
        assert(dut.io.ptw.req.bits.bits.addr === (va >> 12))
        state := fence
      }
    }
    is (fence) {
      dut.io.sfence.valid := true.B
      dut.io.ptw.resp.valid := !delayed
      delay := 0.U
      state := Mux(delayed, late, probe)
    }
    is (late) {
      delay := delay + 1.U
      when (delay === 2.U) {
        dut.io.ptw.resp.valid := true.B
        state := probe
      }
    }
    is (probe) {
      dut.io.req(0).valid := true.B
      assert(dut.io.resp(0).miss, "[dm-test] fenced old PTW response was installed")
      when (dut.io.req(0).fire) { state := newWalk }
    }
    is (newWalk) {
      when (dut.io.ptw.req.fire) { state := refill }
    }
    is (refill) {
      dut.io.ptw.resp.valid := true.B
      response.pte.reserved_for_software := (!oldDM).asUInt
      state := hit
    }
    is (hit) {
      for (w <- 0 until memWidth) {
        dut.io.req(w).valid := true.B
        assert(!dut.io.resp(w).miss && !dut.io.resp(w).pf.ld && !dut.io.resp(w).ae.ld,
          "[dm-test] replacement translation did not hit legally")
        assert(dut.io.resp(w).paddr === pa && dut.io.resp(w).dm === !oldDM,
          "[dm-test] replacement translation returned stale PA/DM")
      }
      SynthesizePrintf("[DM_RACE_PASS] test=%d delayed=%d old_dm=%d new_dm=%d\n",
        test, delayed, oldDM, !oldDM)
      test := test + 1.U
      state := Mux(test === 3.U, done, clear)
    }
  }
}
