package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
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

/**
 * Closes the loop around the real {@link Turret}: the commanded servo power is integrated into a
 * simulated turret angle and fed back through the analog encoder, so the controller drives a
 * (crude) model of the actual mechanism rather than a frozen sensor value.
 *
 * <p>This is what caught the bang-bang bug. The turret buzzed on target because {@code
 * updateTurret} took {@code Math.abs(pidOutput)} and clamped it up to {@code feed_forward}, so
 * every error from the tolerance edge out to tens of degrees commanded the exact same power. No
 * PIDF gains could fix that, which is why tuning and autotuning both appeared to do nothing.
 */
public class TurretLoopDiagnosticTest {

  /** Real FTC control loops run ~20 ms apart; the PIDF's derivative term needs a realistic dt. */
  private static final long LOOP_MS = 20;

  private static final double DEG_PER_LOOP_AT_FULL = 2.4; // ~120 deg/s at full power over 20 ms
  private static final double STICTION = 0.06; // below this power the servo does not move

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

    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);
    turret = new Turret(hardwareMap, mockTelemetry, () -> new Pose(0, 0, 0));
  }

  private void setPhysicalTurretAngle(double deg) {
    var enc = config.turret.analog_encoder;
    double delta = enc.inverted ? -deg / enc.degrees_per_volt : deg / enc.degrees_per_volt;
    voltageBox[0] = enc.zero_voltage + delta;
  }

  /**
   * Signed power in encoder-angle terms: positive means "drives the angle up". Which CRServo
   * direction that corresponds to is configurable, so mirror setTurretPowerRaw's mapping here.
   */
  private double signedCommandedPower() {
    CRServo.Direction increasing =
        config.turret.servo_direction_inverted
            ? CRServo.Direction.FORWARD
            : CRServo.Direction.REVERSE;
    return cmdDir[0] == increasing ? cmdPower[0] : -cmdPower[0];
  }

  /** Steady-state power the controller commands for a fixed error (derivative settled to ~0). */
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
  public void commandedPowerScalesWithErrorInsteadOfBangBang() throws InterruptedException {
    // Goal far away so calculateDynamicTolerance pins to its tightest value.
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    double smallErr = 5.0;
    double midErr = Math.min(15.0, config.turret.travel.max_angle * 0.5);
    double maxErr = config.turret.travel.max_angle;

    double atSmall = steadyStatePowerForError(smallErr, pose);
    double atMid = steadyStatePowerForError(midErr, pose);
    double atMax = steadyStatePowerForError(maxErr, pose);

    // The defining property: bigger error => strictly more power.
    assertTrue(
        "power must grow with error (small=" + atSmall + ", mid=" + atMid + ")", atMid > atSmall);
    assertTrue("power must grow with error (mid=" + atMid + ", max=" + atMax + ")", atMax >= atMid);

    // Small errors must produce gentle correction, not a full-power slam.
    assertTrue(
        "5 deg of error should not command near-max power, got " + atSmall,
        atSmall < config.turret.max_power_output * 0.5);
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
    int reversals = 0;
    double prevSigned = 0;

    for (int i = 0; i < loops; i++) {
      Thread.sleep(LOOP_MS);
      turret.periodic();
      double signed = signedCommandedPower();
      angle += Math.abs(signed) < STICTION ? 0.0 : signed * DEG_PER_LOOP_AT_FULL;
      setPhysicalTurretAngle(angle);

      if (i > 0
          && signed != 0
          && prevSigned != 0
          && Math.signum(signed) != Math.signum(prevSigned)) {
        reversals++;
      }
      if (signed != 0) prevSigned = signed;

      if (i >= settleFrom) {
        minAfterSettle = Math.min(minAfterSettle, angle);
        maxAfterSettle = Math.max(maxAfterSettle, angle);
      }
    }

    // Must actually arrive, and then hold still rather than hunting across the tolerance edge.
    assertEquals("turret should settle on its 20 deg target", 20.0, angle, 1.5);
    assertEquals("settled turret must not oscillate", 0.0, maxAfterSettle - minAfterSettle, 1e-9);
    assertTrue(
        "controller should not reverse direction while converging, got " + reversals,
        reversals <= 2);
  }

  @Test
  public void stallKicksThroughFrictionInsteadOfSittingForever() throws InterruptedException {
    // Simulates real breakaway friction sitting a little above the calibrated feed_forward --
    // exactly what "Turret Done stays false but nothing is moving" looks like. The plain
    // PID+feedforward command is never enough to move the turret on its own; only the stall kick
    // can close the error.
    var t = config.turret;
    var stall = t.stall;
    double baseline = t.ks;
    double kick = stall.kick_power;
    org.junit.Assume.assumeTrue("test requires kick_power > ks", kick > baseline);

    // Pick a stiction threshold strictly below kick_power so the kick always breaks it, and pick
    // the target so that the worst-case (largest-error, start-of-run) PID+ks command
    // still stays under that threshold -- error only shrinks from there without a kick, so if the
    // starting command can't move it, nothing before the kick ever will.
    double realStiction = kick * 0.9;
    org.junit.Assume.assumeTrue("realStiction must still exceed ks", realStiction > baseline);
    double maxSafeError = (realStiction - baseline) / t.pidf.p;
    double dynamicTolerance = turret.calculateDynamicTolerance(10000.0);
    double target = Math.min(maxSafeError * 0.8, t.travel.max_angle - 1.0);
    org.junit.Assume.assumeTrue(
        "target must clear tolerance to stay in the driving branch", target > dynamicTolerance * 2);

    turret.setGoal(10000.0, 0.0);
    turret.setHoldAngle(target);
    turret.setAimMode(Turret.AimMode.HOLD);

    double angle = 0.0;
    setPhysicalTurretAngle(angle);

    // Poll for the kick engaging rather than pre-computing how many loops that should take: real
    // wall-clock sleeps jitter enough (GC, scheduler) that a fixed loop count flakes right at the
    // timeout boundary. Checking the invariant every loop instead -- "unless kicking, the command
    // must never reach stiction" -- is exact regardless of how many loops it actually took.
    boolean sawKick = false;
    boolean movedWithoutKicking = false;
    for (int i = 0; i < 400; i++) {
      Thread.sleep(LOOP_MS);
      turret.periodic();
      boolean kicking = turret.isStallKickActive();
      sawKick |= kicking;

      double signed = signedCommandedPower();
      if (!kicking && Math.abs(signed) >= realStiction) {
        movedWithoutKicking = true;
      }
      if (Math.abs(signed) >= realStiction) {
        angle += signed * DEG_PER_LOOP_AT_FULL;
        setPhysicalTurretAngle(angle);
      }
      if (turret.isTurnDone()) break;
    }

    assertTrue(
        "plain PID+feedforward command must never reach the stiction threshold on its own",
        !movedWithoutKicking);
    assertTrue("stall kick should have engaged at least once", sawKick);
    assertEquals("turret should eventually reach target via the kick", target, angle, 1.0);
  }

  /** Sets the mocked encoder to a raw voltage directly, bypassing the angle conversion. */
  private void setRawVoltage(double volts) {
    voltageBox[0] = volts;
  }

  @Test
  public void invertedDriveIsCaughtBeforeItReachesTheHardStop() throws InterruptedException {
    // Simulates a wrong servo_direction_inverted: the turret moves the OPPOSITE way from the
    // command. Previously the boundary guard watched the end of travel the turret was moving away
    // from, so nothing stopped it -- it drove through the hard stop and skipped gears.
    turret.setGoal(10000.0, 0.0);
    turret.setHoldAngle(20.0);
    turret.setAimMode(Turret.AimMode.HOLD);

    double angle = 0.0;
    setPhysicalTurretAngle(angle);

    double worstAngle = 0.0;
    for (int i = 0; i < 60; i++) {
      Thread.sleep(LOOP_MS);
      turret.periodic();
      double signed = signedCommandedPower();
      // Backwards on purpose: the actuator is wired the wrong way round.
      angle -= Math.abs(signed) < STICTION ? 0.0 : signed * DEG_PER_LOOP_AT_FULL;
      setPhysicalTurretAngle(angle);
      worstAngle = Math.min(worstAngle, angle);
    }

    assertTrue("an inverted drive must be detected", turret.isRunawayFaulted());
    assertEquals("turret must be stopped once runaway is detected", 0.0, cmdPower[0], 1e-9);

    // It must be caught well before the mechanical stop, not after chewing into it.
    double stop = config.turret.travel.min_angle;
    assertTrue(
        "runaway should trip before the hard stop (worst " + worstAngle + " vs stop " + stop + ")",
        worstAngle > stop);
  }

  @Test
  public void angleIsContinuousAcrossThePotentiometerWrapPoint() {
    // Real measured values from the robot: hard stops at 0.183 V and 1.836 V, and the turret's
    // true physical center sitting at 2.67 V -- i.e. OUTSIDE the two stop voltages, because the
    // travel crosses the pot's wrap point. Read linearly, true center reported -70 deg.
    var enc = config.turret.analog_encoder;
    org.junit.Assume.assumeTrue("wrap handling must be enabled", enc.full_scale_voltage > 0);

    double center = enc.zero_voltage;
    assertEquals("center must read zero", 0.0, turret.angleForVoltage(center), 0.5);

    double atLowStop = turret.angleForVoltage(enc.min_voltage);
    double atHighStop = turret.angleForVoltage(enc.max_voltage);

    // Both stops must land on opposite sides of center with near-equal magnitude -- the signature
    // of a correctly unwrapped circular scale. Linear math put them 0 and -70 instead.
    assertTrue(
        "stops must straddle center, got " + atLowStop + " and " + atHighStop,
        Math.signum(atLowStop) != Math.signum(atHighStop));
    assertEquals(
        "stops must be symmetric about center", Math.abs(atLowStop), Math.abs(atHighStop), 2.0);

    // And both must sit within the configured travel plus its fault margin, or the turret would
    // fault the instant it reached one of its own hard stops.
    var travel = config.turret.travel;
    double limit = travel.max_angle + Turret.FAULT_MARGIN_DEG;
    assertTrue("stop " + atLowStop + " must be inside +/-" + limit, Math.abs(atLowStop) <= limit);
    assertTrue("stop " + atHighStop + " must be inside +/-" + limit, Math.abs(atHighStop) <= limit);
  }

  @Test
  public void steadyWrappedVoltageIsRejectedRatherThanDrivenOn() throws InterruptedException {
    // The real failure: the pot wrapped past its rail and reported a wrong but perfectly STEADY
    // angle. A loop-to-loop jump filter sees nothing wrong with a steady value, so the controller
    // drove full power toward a target physically behind a hard stop and skipped gears.
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    // A voltage that unwraps to an angle well outside the mechanical travel.
    var travel = config.turret.travel;
    var enc = config.turret.analog_encoder;
    double bogusAngle = travel.max_angle + Turret.FAULT_MARGIN_DEG + 15.0;
    double wrapped = enc.zero_voltage - bogusAngle / enc.degrees_per_volt;

    // Hold the bogus voltage perfectly steady across several loops -- no jump to detect.
    turret.setTargetTurnAngle(5.0);
    for (int i = 0; i < 4; i++) {
      setRawVoltage(wrapped);
      Thread.sleep(LOOP_MS);
      turret.updateTurret(pose);
    }

    assertTrue(
        "a voltage outside the calibrated window must be flagged", turret.isEncoderFaulted());
    assertEquals("turret must not be driven on a wrapped reading", 0.0, cmdPower[0], 1e-9);
  }

  @Test
  public void rejectedJumpDoesNotFreezeControllerOnStaleAngle() throws InterruptedException {
    // Previously the jump filter only refreshed its history when NOT faulted, so one rejected
    // sample latched the controller onto a stale angle forever -- the robot showed
    // "Turret Relative: -109" while the live sensor read +23.73.
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);
    turret.setTargetTurnAngle(0.0);

    setPhysicalTurretAngle(0.0);
    Thread.sleep(LOOP_MS);
    turret.updateTurret(pose);

    // One implausible sample (still inside the valid voltage window so only the jump filter acts).
    double jumped = Turret.MAX_JUMP_DEG + 5.0;
    setPhysicalTurretAngle(jumped);
    Thread.sleep(LOOP_MS);
    turret.updateTurret(pose);

    // Next loop the same reading persists -- it must now be accepted, not rejected forever.
    setPhysicalTurretAngle(jumped);
    Thread.sleep(LOOP_MS);
    turret.updateTurret(pose);

    assertEquals(
        "controller must re-sync to the persisting reading instead of freezing",
        jumped,
        turret.getCurrentTurnAngle(),
        0.5);
    assertTrue(
        "a persisting in-window reading must not leave a latched fault",
        !turret.isEncoderFaulted());
  }

  @Test
  public void encoderRangeWarningFlagsUnreachableTravelLimit() {
    // A travel limit outside what the encoder can report is silently unenforceable -- exactly the
    // state the robot's own config was in, which let the turret grind into its hard stop.
    double[] range = turret.getReachableAngleRange();
    assertTrue("reachable range should be ordered", range[0] < range[1]);

    var travel = config.turret.travel;
    boolean limitsReachable = range[0] <= travel.min_angle && range[1] >= travel.max_angle;
    if (limitsReachable) {
      assertNull(turret.getEncoderRangeWarning());
    } else {
      assertNotNull(
          "unreachable travel limits must be reported, reachable=["
              + range[0]
              + ", "
              + range[1]
              + "] travel=["
              + travel.min_angle
              + ", "
              + travel.max_angle
              + "]",
          turret.getEncoderRangeWarning());
    }
  }

  @Test
  public void encoderJumpIsRejectedInsteadOfSaturatingTheController() throws InterruptedException {
    turret.setGoal(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    // Establish a stable reading first.
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(0.0);
    Thread.sleep(LOOP_MS);
    turret.updateTurret(pose);

    // Now simulate the potentiometer wrapping past its electrical rail: a single-loop step far
    // larger than the turret could physically travel.
    setPhysicalTurretAngle(Turret.MAX_JUMP_DEG + 50.0);
    Thread.sleep(LOOP_MS);
    turret.updateTurret(pose);

    // The controller must refuse the sample and cut power rather than chase a bogus 80 deg error.
    assertEquals("power must be cut on an implausible encoder jump", 0.0, cmdPower[0], 1e-9);
  }
}
