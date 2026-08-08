package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

/** Diagnostic unit test suite verifying closed-loop control of the simplified {@link Turret}. */
public class TurretLoopDiagnosticTest {

  private static final long LOOP_MS = 20;
  private static final double DEG_PER_LOOP_AT_FULL = 2.4;
  private static final double STICTION = 0.06;

  private Turret turret;
  private double[] voltageBox;
  private double[] cmdPower;
  private CRServo.Direction[] cmdDir;

  @Before
  public void setUp() {
    HardwareMap hardwareMap = Mockito.mock(HardwareMap.class);
    CRServo mockServo = Mockito.mock(CRServo.class);
    DcMotorEx mockMotor = Mockito.mock(DcMotorEx.class);
    Mockito.when(hardwareMap.get(CRServo.class, "turn")).thenReturn(mockServo);

    cmdPower = new double[] {0.0};
    cmdDir = new CRServo.Direction[] {CRServo.Direction.FORWARD};
    Mockito.doAnswer(
            inv -> {
              cmdPower[0] = inv.getArgument(0);
              return null;
            })
        .when(mockServo)
        .setPower(Mockito.anyDouble());
    Mockito.doAnswer(
            inv -> {
              cmdDir[0] = inv.getArgument(0);
              return null;
            })
        .when(mockServo)
        .setDirection(Mockito.any());
    Mockito.when(mockServo.getPower()).thenAnswer(inv -> cmdPower[0]);

    AnalogInput mockAnalog = Mockito.mock(AnalogInput.class);
    voltageBox = new double[] {config.turret.analog_encoder.zero_voltage};
    Mockito.when(hardwareMap.get(AnalogInput.class, "turnanalog")).thenReturn(mockAnalog);
    Mockito.when(mockAnalog.getVoltage()).thenAnswer(inv -> voltageBox[0]);

    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftBack")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightBack")).thenReturn(mockMotor);

    // config.turret.enabled defaults to false on a freshly-built robot; this suite exercises the
    // closed-loop control math, not the enabled flag, so force it on regardless of test order.
    config.turret.enabled = true;

    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);
    turret = new Turret(hardwareMap, mockTelemetry, () -> new Pose(0, 0, 0));
  }

  private void setPhysicalTurretAngle(double deg) {
    var enc = config.turret.analog_encoder;
    double delta = enc.inverted ? -deg / enc.degrees_per_volt : deg / enc.degrees_per_volt;
    voltageBox[0] = enc.zero_voltage + delta;
  }

  private double signedCommandedPower() {
    return cmdPower[0];
  }

  private double steadyStatePowerForError(double errorDeg, Pose pose) throws InterruptedException {
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(errorDeg);
    Thread.sleep(LOOP_MS);
    turret.updateTurret(pose);
    setPhysicalTurretAngle(0.0);
    Thread.sleep(LOOP_MS);
    turret.updateTurret(pose);
    return signedCommandedPower();
  }

  @Test
  public void commandedPowerScalesWithError() throws InterruptedException {
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    double smallErr = 5.0;
    double midErr = Math.min(15.0, config.turret.travel.max_angle * 0.5);
    double maxErr = config.turret.travel.max_angle;

    double atSmall = steadyStatePowerForError(smallErr, pose);
    double atMid = steadyStatePowerForError(midErr, pose);
    double atMax = steadyStatePowerForError(maxErr, pose);

    assertTrue(
        "power must grow with error (small=" + atSmall + ", mid=" + atMid + ")", atMid > atSmall);
    assertTrue("power must grow with error (mid=" + atMid + ", max=" + atMax + ")", atMax >= atMid);
    // Deliberately not asserting an absolute power for a given error: that is a statement about
    // how aggressively the turret happens to be tuned, and it fails whenever p is legitimately
    // retuned. What must hold regardless of gains is that power never leaves the clamp.
    for (double p : new double[] {atSmall, atMid, atMax}) {
      assertTrue(
          "commanded power must stay within max_power_output, got " + p,
          Math.abs(p) <= config.turret.max_power_output + 1e-9);
    }
  }

  @Test
  public void closedLoopHoldConvergesWithoutOscillating() throws InterruptedException {
    turret.setGoal(10000.0, 0.0);
    turret.setHoldAngle(20.0);
    turret.setAimMode(Turret.AimMode.HOLD);

    double angle = 0.0;
    setPhysicalTurretAngle(angle);

    final int loops = 120;
    final int settleFrom = 80;
    double minAfterSettle = Double.MAX_VALUE;
    double maxAfterSettle = -Double.MAX_VALUE;

    for (int i = 0; i < loops; i++) {
      Thread.sleep(LOOP_MS);
      turret.periodic();
      double signed = signedCommandedPower();
      angle += Math.abs(signed) < STICTION ? 0.0 : signed * DEG_PER_LOOP_AT_FULL;
      setPhysicalTurretAngle(angle);

      if (i >= settleFrom) {
        minAfterSettle = Math.min(minAfterSettle, angle);
        maxAfterSettle = Math.max(maxAfterSettle, angle);
      }
    }

    assertEquals("turret should settle near its 20 deg target", 20.0, angle, 3.5);
    assertEquals("settled turret must not oscillate", 0.0, maxAfterSettle - minAfterSettle, 1e-9);
  }

  /**
   * Guards the live-tuner bug: {@code PIDFController.run()} re-reads its gains from the supplier it
   * was constructed with, so handing it a brand new coefficients object had no effect and the
   * turret kept driving on the gains baked in at construction even with every dashboard gain at 0.
   */
  @Test
  public void zeroedGainsFromTheLiveTunerStopTheTurret() throws InterruptedException {
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    double movingPower = steadyStatePowerForError(config.turret.travel.max_angle, pose);
    assertTrue("baseline: config gains must actually drive the turret", Math.abs(movingPower) > 0);

    // kS is added outside the PIDF, so zeroing the gains leaves exactly the stiction term.
    turret.setPIDF(0.0, 0.0, 0.0, 0.0);
    turret.setKs(0.0, 0.0);
    double zeroedPower = steadyStatePowerForError(config.turret.travel.max_angle, pose);

    assertEquals("all-zero gains must command no power", 0.0, zeroedPower, 1e-9);
  }

  /** kS is a directional stiction term added on top of the PIDF output. */
  @Test
  public void ksAloneDrivesTheTurretTowardTheTarget() throws InterruptedException {
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    turret.setPIDF(0.0, 0.0, 0.0, 0.0);
    turret.setKs(0.08, 0.05);

    double positivePower = steadyStatePowerForError(config.turret.travel.max_angle, pose);
    assertEquals("a positive target must command +ks_positive", 0.08, positivePower, 1e-9);

    double negativePower = steadyStatePowerForError(config.turret.travel.min_angle, pose);
    assertEquals("a negative target must command -ks_negative", -0.05, negativePower, 1e-9);
  }

  /**
   * kS must not also be routed through the PIDF's F term. {@code run()} computes {@code
   * feedForwardInput * F}, and the loop already adds the same kS to the output separately, so a
   * non-zero f silently double-counts it.
   */
  @Test
  public void nonZeroFGainDoesNotDoubleCountKs() throws InterruptedException {
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    turret.setKs(0.08, 0.08);
    turret.setPIDF(0.0, 0.0, 0.0, 0.0);
    double withoutF = steadyStatePowerForError(config.turret.travel.max_angle, pose);

    turret.setPIDF(0.0, 0.0, 0.0, 0.05);
    double withF = steadyStatePowerForError(config.turret.travel.max_angle, pose);

    assertEquals("f must add its own constant, not scale kS by f", withoutF + 0.05, withF, 1e-9);
  }

  /**
   * An out-of-range reading must cut power in BOTH directions. Deciding which way is "back toward
   * travel" needs the encoder angle, and this is precisely the case where that reading cannot be
   * trusted -- letting the loop auto-recover here drove the turret into its hard stop.
   */
  @Test
  public void encoderReadingOutsideFaultWindowCutsPowerInBothDirections() {
    for (double faulted :
        new double[] {
          config.turret.travel.min_angle - Turret.FAULT_MARGIN_DEG - 2.0,
          config.turret.travel.max_angle + Turret.FAULT_MARGIN_DEG + 2.0
        }) {
      setPhysicalTurretAngle(faulted);
      assertTrue("test setup must be outside the fault window", !turret.isReadingWithinTravel());

      for (double power : new double[] {0.4, -0.4}) {
        turret.setTurretPowerRaw(power);
        assertEquals(
            "power must be cut at " + faulted + " deg for command " + power,
            0.0,
            signedCommandedPower(),
            1e-9);
        assertNotNull("blocking must be reported", turret.getPowerBlockedReason());
      }
    }
  }

  /**
   * The protection that was missing when the turret was driven into its hard stop: every
   * position-based guard trusts the encoder, so none of them fire when the encoder is the thing
   * that is wrong. Lack of progress under power does not depend on the reading being correct.
   */
  @Test
  public void stallWatchdogCutsPowerWhenDrivenWithoutMoving() throws InterruptedException {
    var stall = config.turret.stall;
    org.junit.Assume.assumeTrue("stall watchdog must be enabled", stall.enabled);

    // Well inside the valid window, so only the watchdog can stop this.
    setPhysicalTurretAngle(0.0);
    double power = Math.max(stall.power_threshold + 0.05, 0.2);

    long deadline = System.nanoTime() + (long) ((stall.timeout_sec + 0.5) * 1e9);
    while (System.nanoTime() < deadline && !turret.isFaulted()) {
      turret.setTurretPowerRaw(power); // angle never changes -> no progress
      Thread.sleep(10);
    }

    assertTrue("watchdog must latch a stall", turret.isFaulted());
    assertEquals("stalled turret must be unpowered", 0.0, signedCommandedPower(), 1e-9);
    assertNotNull("stall must be reported", turret.getPowerBlockedReason());

    turret.clearFault();
    turret.setTurretPowerRaw(power);
    assertEquals("clearFault must restore control", power, signedCommandedPower(), 1e-9);
  }

  @Test
  public void stallWatchdogDoesNotTripWhileTheTurretIsMoving() throws InterruptedException {
    var stall = config.turret.stall;
    org.junit.Assume.assumeTrue("stall watchdog must be enabled", stall.enabled);

    double power = Math.max(stall.power_threshold + 0.05, 0.2);
    double angle = 0.0;
    double step = Math.max(stall.min_progress_deg * 2.0, 0.1);

    long deadline = System.nanoTime() + (long) ((stall.timeout_sec * 3.0) * 1e9);
    while (System.nanoTime() < deadline) {
      angle += step;
      if (angle > config.turret.travel.max_angle) {
        angle = config.turret.travel.min_angle;
      }
      setPhysicalTurretAngle(angle);
      turret.setTurretPowerRaw(power);
      Thread.sleep(10);
    }

    assertTrue("a moving turret must never be flagged stalled", !turret.isFaulted());
  }

  @Test
  public void angleIsContinuousAcrossThePotentiometerWrapPoint() {
    var enc = config.turret.analog_encoder;
    org.junit.Assume.assumeTrue("wrap handling must be enabled", enc.full_scale_voltage > 0);

    double center = enc.zero_voltage;
    assertEquals("center must read zero", 0.0, turret.angleForVoltage(center), 0.5);

    double atLowStop = turret.angleForVoltage(enc.min_voltage);
    double atHighStop = turret.angleForVoltage(enc.max_voltage);

    assertTrue(
        "stops must straddle center, got " + atLowStop + " and " + atHighStop,
        Math.signum(atLowStop) != Math.signum(atHighStop));
    assertEquals(
        "stops must be symmetric about center", Math.abs(atLowStop), Math.abs(atHighStop), 2.0);

    var travel = config.turret.travel;
    double limit = travel.max_angle + Turret.FAULT_MARGIN_DEG;
    assertTrue("stop " + atLowStop + " must be inside +/-" + limit, Math.abs(atLowStop) <= limit);
    assertTrue("stop " + atHighStop + " must be inside +/-" + limit, Math.abs(atHighStop) <= limit);
  }

  @Test
  public void encoderRangeWarningFlagsUnreachableTravelLimit() {
    double[] range = turret.getReachableAngleRange();
    assertTrue("reachable range should be ordered", range[0] < range[1]);

    var travel = config.turret.travel;
    boolean limitsReachable = range[0] <= travel.min_angle && range[1] >= travel.max_angle;
    if (limitsReachable) {
      String warning = turret.getEncoderRangeWarning();
      if (warning != null) {
        assertFalse(
            "travel limits should not be reported unreachable when reachable",
            warning.contains("unreachable"));
      }
    } else {
      assertNotNull("unreachable travel limits must be reported", turret.getEncoderRangeWarning());
    }
  }

  /**
   * An inverted loop (servo_direction_inverted disagreeing with analog_encoder.inverted) drives the
   * turret away from its target and into a hard stop, with every reading staying plausible the
   * whole way. Only the growing error gives it away.
   */
  @Test
  public void runawayWatchdogCutsPowerWhenErrorGrowsUnderPower() throws InterruptedException {
    org.junit.Assume.assumeTrue("runaway watchdog must be enabled", config.turret.runaway.enabled);
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    turret.setTargetTurnAngle(config.turret.travel.max_angle);

    // Simulate an inverted plant: the turret walks away from the target instead of toward it.
    double angle = 0.0;
    for (int i = 0; i < 200 && !turret.isFaulted(); i++) {
      setPhysicalTurretAngle(angle);
      Thread.sleep(5);
      turret.updateTurret(pose);
      angle = Math.max(angle - 1.0, config.turret.travel.min_angle);
    }

    assertTrue("watchdog must latch on a diverging error", turret.isFaulted());
    assertEquals("runaway turret must be unpowered", 0.0, signedCommandedPower(), 1e-9);
    assertTrue(
        "fault must name the inverted-loop cause, got " + turret.getFaultReason(),
        turret.getFaultReason().contains("RUNAWAY"));
  }

  @Test
  public void runawayWatchdogToleratesNormalConvergence() throws InterruptedException {
    org.junit.Assume.assumeTrue("runaway watchdog must be enabled", config.turret.runaway.enabled);
    turret.setGoal(10000.0, 0.0);
    turret.setHoldAngle(20.0);
    turret.setAimMode(Turret.AimMode.HOLD);

    double angle = 0.0;
    setPhysicalTurretAngle(angle);
    for (int i = 0; i < 120; i++) {
      Thread.sleep(LOOP_MS);
      turret.periodic();
      double signed = signedCommandedPower();
      angle += Math.abs(signed) < STICTION ? 0.0 : signed * DEG_PER_LOOP_AT_FULL;
      setPhysicalTurretAngle(angle);
    }

    assertTrue(
        "a converging turret must never fault, got " + turret.getFaultReason(),
        !turret.isFaulted());
  }
}
