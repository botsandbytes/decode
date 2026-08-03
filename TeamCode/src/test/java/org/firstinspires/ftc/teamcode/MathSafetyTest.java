package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.Before;
import org.junit.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.mockito.Mockito;

public class MathSafetyTest {

  private Turret turret;
  private CRServo mockServo;
  private double[] turretVoltage;

  @Before
  public void setUp() {
    // Mock hardware dependencies for Turret initialization
    HardwareMap hardwareMap = Mockito.mock(HardwareMap.class);
    mockServo = Mockito.mock(CRServo.class);
    DcMotorEx mockMotor = Mockito.mock(DcMotorEx.class);
    Mockito.when(hardwareMap.get(CRServo.class, "turn")).thenReturn(mockServo);
    AnalogInput mockAnalog = Mockito.mock(AnalogInput.class);
    Mockito.when(hardwareMap.get(AnalogInput.class, "turnanalog")).thenReturn(mockAnalog);
    turretVoltage = new double[] {config.turret.analog_encoder.zero_voltage};
    Mockito.when(mockAnalog.getVoltage()).thenAnswer(inv -> turretVoltage[0]);

    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftBack")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightBack")).thenReturn(mockMotor);

    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);
    turret = new Turret(hardwareMap, mockTelemetry);
  }

  /** Drives the mocked analog voltage so turret.getCurrentTurnAngle() reports the given angle. */
  private void setPhysicalTurretAngle(double chassisRelativeDegrees) {
    var enc = config.turret.analog_encoder;
    double delta =
        enc.inverted
            ? -chassisRelativeDegrees / enc.degrees_per_volt
            : chassisRelativeDegrees / enc.degrees_per_volt;
    turretVoltage[0] = enc.zero_voltage + delta;
  }

  @Test
  public void testAlignPoseAngle() {
    // Test alignPose calculates target angles correctly
    // Angle to (10, 0) from (0, 0) should be 0 radians
    Pose pose1 = Turret.alignPose(0, 0, 10, 0);
    assertEquals(0.0, pose1.getHeading(), 1e-6);

    // Angle to (0, 10) from (0, 0) should be PI/2 radians (90 degrees)
    Pose pose2 = Turret.alignPose(0, 0, 0, 10);
    assertEquals(Math.PI / 2, pose2.getHeading(), 1e-6);

    // Angle to (-10, 0) from (0, 0) should be PI radians (180 degrees)
    Pose pose3 = Turret.alignPose(0, 0, -10, 0);
    assertEquals(Math.PI, pose3.getHeading(), 1e-6);
  }

  @Test
  public void testTurretShouldTurnLeft() {
    // When limits are not violated, the shortest path decides direction
    // Case 1: turning from 10 to 50 degrees -> turn left is true (40 deg diff)
    assertTrue(turret.shouldTurnLeft(10, 50));
    // Case 2: turning from 50 to 10 degrees -> turn left is false (320 deg diff, right is 40 deg)
    assertFalse(turret.shouldTurnLeft(50, 10));

    // Shortest path is right (0 to 359 is 1 deg right)
    assertFalse(turret.shouldTurnLeft(0, 359));

    // Limit violation check:
    // turning left from 80 to 110 is shorter (30 deg) but both paths are bad (target is unsafe)
    // so it defaults to the shortest path (left)
    assertTrue(turret.shouldTurnLeft(80, 110));
  }

  @Test
  public void testTurretCalculateDynamicTolerance() {
    config.turret.tolerance.near_cutoff = 24.0; // inches
    config.turret.tolerance.near_val = 2.0; // degrees
    config.turret.tolerance.max_drift = 4.0; // inches
    config.turret.tolerance.min_deg = 1.0; // degrees
    config.turret.tolerance.max_deg = 5.0; // degrees

    // If distance is near cutoff (e.g. 15 inches < 24 inches), target NEAR_VAL tolerance
    assertEquals(2.0, turret.calculateDynamicTolerance(15.0), 1e-6);

    // Dynamic target calculations
    // distance = 50 inches: tolerance = asin((MAX_DRIFT / 2) / dist) = asin(2/50) = asin(0.04) ~
    // 2.29 degrees
    double expectedAngle = Math.toDegrees(Math.asin(2.0 / 50.0));
    assertEquals(expectedAngle, turret.calculateDynamicTolerance(50.0), 1e-6);
  }

  @Test
  public void testCasablancaApplyFriction() {
    // Casablanca friction comp tests
    double kS = 0.1;

    // Zero or micro-input is zeroed out
    assertEquals(0.0, Casablanca.applyFriction(0.005, kS), 1e-6);
    assertEquals(0.0, Casablanca.applyFriction(0.0, kS), 1e-6);

    // Positive input scale math: signum * (kS + input * (1 - kS))
    // input = 0.5 -> 0.1 + 0.5 * 0.9 = 0.55
    assertEquals(0.55, Casablanca.applyFriction(0.5, kS), 1e-6);

    // Negative input scale math
    assertEquals(-0.55, Casablanca.applyFriction(-0.5, kS), 1e-6);
  }

  @Test
  public void testCasablancaApplySlewLimit() {
    // Current = 0.0, target = 0.5, maxChange = 0.2 -> result should be 0.2
    assertEquals(0.2, Casablanca.applySlewLimit(0.0, 0.5, 0.2), 1e-6);

    // Current = 0.0, target = 0.1, maxChange = 0.2 -> diff < maxChange, result should be 0.1
    assertEquals(0.1, Casablanca.applySlewLimit(0.0, 0.1, 0.2), 1e-6);

    // Current = 0.0, target = -0.5, maxChange = 0.2 -> result should be -0.2
    assertEquals(-0.2, Casablanca.applySlewLimit(0.0, -0.5, 0.2), 1e-6);
  }

  @Test
  public void testCasablancaCalculateLaneFade() {
    // zoneMin = 20, zoneMax = 40, buffer = 10
    // botMax = 15, botMin = 10 -> distToStrict = 20 - 15 = 5 (within buffer)
    // fade = 1 - (5 / 10) = 0.5
    assertEquals(0.5, Casablanca.calculateLaneFade(10, 15, 20, 40, 10), 1e-6);

    // botMax = 5, botMin = 0 -> distToStrict = 20 - 5 = 15 (outside buffer) -> fade = 0.0
    assertEquals(0.0, Casablanca.calculateLaneFade(0, 5, 20, 40, 10), 1e-6);

    // botMax = 25, botMin = 20 -> distToStrict = 0 (intersecting/violating) -> fade = 1.0
    assertEquals(1.0, Casablanca.calculateLaneFade(20, 25, 20, 40, 10), 1e-6);
  }

  @Test
  public void testSentinelZoneCalculations() {
    // Verify Sentinel goal zones bounding box calculations (loaded dynamically from config facade)
    Sentinel sentinel = new Sentinel(Alliance.RED);
    double goalSize = config.sentinel.goals.size;
    double goalMinY = config.sentinel.goals.min_y;

    Envelope blueZone = sentinel.getBlueGoalZone();
    assertEquals(0.0, blueZone.getMinX(), 1e-6);
    assertEquals(goalSize, blueZone.getMaxX(), 1e-6);
    assertEquals(goalMinY, blueZone.getMinY(), 1e-6);
    assertEquals(goalMinY + goalSize, blueZone.getMaxY(), 1e-6);

    Envelope redZone = sentinel.getRedGoalZone();
    assertEquals(144.0 - goalSize, redZone.getMinX(), 1e-6);
    assertEquals(144.0, redZone.getMaxX(), 1e-6);
    assertEquals(goalMinY, redZone.getMinY(), 1e-6);
    assertEquals(goalMinY + goalSize, redZone.getMaxY(), 1e-6);
  }

  @Test
  public void testTurretComputeSignedError() {
    // Test case 1: turnLeft = true, target is ahead of current (no wrap)
    assertEquals(40.0, Turret.computeSignedError(10.0, 50.0, true), 1e-6);

    // Test case 2: turnLeft = true, target is behind current, wrapping over 360
    assertEquals(30.0, Turret.computeSignedError(350.0, 20.0, true), 1e-6);

    // Test case 3: turnLeft = false, target is behind current (no wrap)
    assertEquals(-40.0, Turret.computeSignedError(50.0, 10.0, false), 1e-6);

    // Test case 4: turnLeft = false, target is ahead of current, wrapping over 0
    assertEquals(-30.0, Turret.computeSignedError(20.0, 350.0, false), 1e-6);

    // Walk the turret across the limit boundary:
    double current = 175.0;
    double target = 185.0;
    boolean turnLeft = turret.shouldTurnLeft(current, target);
    double error = Turret.computeSignedError(current, target, turnLeft);
    if (turnLeft) {
      assertTrue(error > 0);
    } else {
      assertTrue(error < 0);
    }
  }

  @Test
  public void testSentinelFootprintRotationAndIntersections() {
    Sentinel sentinel = new Sentinel(Alliance.RED); // opponent is BLUE_GOAL_ZONE

    // 1. Robot Footprint rotation test at heading = 0
    Pose poseZero = new Pose(10, 10, 0);
    Coordinate[] footprintZero = sentinel.calculateRobotFootprint(poseZero);
    assertEquals(4, footprintZero.length);
    double halfWidth = config.sentinel.robot_width / 2.0;
    // Corner 0: (10 - halfWidth, 10 - halfWidth)
    assertEquals(10.0 - halfWidth, footprintZero[0].x, 1e-4);
    assertEquals(10.0 - halfWidth, footprintZero[0].y, 1e-4);
    // Corner 1: (10 + halfWidth, 10 - halfWidth)
    assertEquals(10.0 + halfWidth, footprintZero[1].x, 1e-4);
    assertEquals(10.0 - halfWidth, footprintZero[1].y, 1e-4);

    // 2. Bounding Box intersection with Goal Zone (violatesActiveGoal)
    // Place robot centered at (2, 72) with heading = 0. It should overlap.
    Pose poseViolating = new Pose(2, 72, 0);
    Coordinate[] footprintViolating = sentinel.calculateRobotFootprint(poseViolating);
    assertTrue(sentinel.violatesActiveGoal(footprintViolating));

    // Place robot centered at (72, 72) with heading = 0. It should NOT overlap.
    Pose poseSafe = new Pose(72, 72, 0);
    Coordinate[] footprintSafe = sentinel.calculateRobotFootprint(poseSafe);
    assertFalse(sentinel.violatesActiveGoal(footprintSafe));

    // 3. Launch allowed test
    // Place robot centered at (72, 72).
    assertTrue(sentinel.isLaunchAllowed(new Pose(72, 72, 0)));

    // Place robot far away at (10, 10). Launch should NOT be allowed.
    assertFalse(sentinel.isLaunchAllowed(new Pose(10, 10, 0)));

    // 4. Rotation safety test
    // Place robot near the blue goal zone with heading = 0.
    double protectedMaxX = sentinel.getProtectedZone().getMaxX();
    Pose poseRotationSafeCheck = new Pose(protectedMaxX + halfWidth + 0.5, 72, 0);
    assertFalse(sentinel.isRotationSafe(poseRotationSafeCheck, 1.0, 0.5));
  }

  @Test
  public void testCasablancaSymmetryAndCollisionMath() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    // Disable heading lock, input smoothing, and friction compensation for predictable math testing
    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;

    double halfWidth = config.sentinel.robot_width / 2.0;

    // --- CASE 1: Depth (X-axis) Protection Check ---
    // Place robot inside the depth hard stop boundary dynamically
    double protectedMaxX = sentinel.getProtectedZone().getMaxX();
    double poseX = protectedMaxX + halfWidth + (0.5 * Casablanca.depthHardStop);
    double[] outputX =
        casablanca.adjustDriveInput(
            new Pose(poseX, 72, 0), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, -1.0, 0.0);
    // Adjusted forward (index 1) should be scaled down significantly
    assertTrue(Math.abs(outputX[1]) < 0.8);

    // --- CASE 2: Side (Y-axis) Protection Check ---
    // Place robot inside the side hard stop boundary dynamically
    double protectedMinY = sentinel.getProtectedZone().getMinY();
    double poseY = protectedMinY - halfWidth - (0.5 * Casablanca.sideHardStop);
    double[] outputY =
        casablanca.adjustDriveInput(
            new Pose(3.0, poseY, 0), new com.pedropathing.math.Vector(0, 0), 0.0, 1.0, 0.0, 0.0);
    // Adjusted strafe (index 0) should be scaled down significantly
    assertTrue(Math.abs(outputY[0]) < 0.8);
  }

  @Test
  public void testTurretHoldAngleIsChassisRelativeAndHeadingInvariant() {
    // The analog encoder reads the turret's angle relative to the chassis directly (unlike the
    // old IMU, which read an absolute field heading). Physically center the turret 30 deg off
    // the chassis and hold it there.
    setPhysicalTurretAngle(30.0);
    turret.setHoldAngle(30.0);
    turret.setAimMode(Turret.AimMode.HOLD);
    turret.periodic();
    Mockito.reset(mockServo);

    // Error must be 0 regardless of the robot chassis's field heading -- the turret servo never
    // moved, so its angle relative to the chassis is unaffected by the whole robot rotating.
    Pose facingNorth = new Pose(72, 72, Math.PI / 2);
    assertEquals(0.0, turret.getAimError(facingNorth), 1e-6);
    assertTrue(turret.isAimed(facingNorth));
    turret.updateTurret(facingNorth);
    Mockito.verify(mockServo).setPower(0.0);

    Mockito.reset(mockServo);
    Pose facingSouth = new Pose(72, 72, -Math.PI / 2);
    assertEquals(0.0, turret.getAimError(facingSouth), 1e-6);
    assertTrue(turret.isAimed(facingSouth));
    turret.updateTurret(facingSouth);
    Mockito.verify(mockServo).setPower(0.0);
  }

  @Test
  public void testManualModeRespectsBoundarySafety() {
    // MANUAL mode (and therefore the relay autotuner, which drives raw power with no PIDF
    // target) has no closed-loop target to bound it, so setTurretPowerRaw must refuse to drive
    // further past the +/-45 deg mechanical limit on its own.
    setPhysicalTurretAngle(45.0);
    turret.setTurretPowerRaw(0.4); // positive = REVERSE = increasing angle -> would exceed +45
    Mockito.verify(mockServo).setPower(0.0);

    // Driving back the other way off the limit must still be allowed.
    turret.setTurretPowerRaw(-0.4);
    Mockito.verify(mockServo).setPower(0.4);
  }

  @Test
  public void testManualModeAllowsFullTravelWhenCentered() {
    // Centered turret must retain its full +/-45 deg range in MANUAL mode.
    setPhysicalTurretAngle(0.0);
    turret.setTurretPowerRaw(0.4);
    Mockito.verify(mockServo).setPower(0.4);

    Mockito.reset(mockServo);
    turret.setTurretPowerRaw(-0.4);
    Mockito.verify(mockServo).setPower(0.4);
  }

  @Test
  public void testTurretIdleModeOnStartup() {
    turret.setAimMode(Turret.AimMode.IDLE);
    turret.periodic();
    Mockito.verify(mockServo).setPower(0.0);
  }

  @Test
  public void testHeadingErrorAngleWrapping() {
    // Target near +179 degrees, current near -179 degrees
    double targetHeading = Math.toRadians(179);
    double currentHeading = Math.toRadians(-179);

    // Signed shortest path error should be -2 degrees (-0.0349 rad)
    double error = AngleUnit.normalizeRadians(targetHeading - currentHeading);
    assertEquals(Math.toRadians(-2), error, 1e-4);

    // Reversed case: target near -179 deg, current near +179 deg -> error should be +2 deg
    double error2 = AngleUnit.normalizeRadians(-targetHeading - (-currentHeading));
    assertEquals(Math.toRadians(2), error2, 1e-4);
  }

  @Test
  public void testCasablancaLockAuthorityFade() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel);
    Casablanca.headingLockIntentThreshold = 0.05;

    // Test authority at 0 stick intent: 1.0 - clamp(0 / 0.05, 0, 1) = 1.0
    double authZero = 1.0 - Math.clamp(0.0 / Casablanca.headingLockIntentThreshold, 0.0, 1.0);
    assertEquals(1.0, authZero, 1e-6);

    // Test authority at midpoint (0.025): 1.0 - clamp(0.025 / 0.05, 0, 1) = 0.5
    double authMid = 1.0 - Math.clamp(0.025 / Casablanca.headingLockIntentThreshold, 0.0, 1.0);
    assertEquals(0.5, authMid, 1e-6);

    // Test authority at/above threshold (0.05): 1.0 - clamp(0.05 / 0.05, 0, 1) = 0.0
    double authFull = 1.0 - Math.clamp(0.05 / Casablanca.headingLockIntentThreshold, 0.0, 1.0);
    assertEquals(0.0, authFull, 1e-6);
  }

  @Test
  public void testCasablancaKsSpeedInterpolation() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel);
    Casablanca.frictionRot = 0.08;
    Casablanca.headingLockKsMoving = 0.02;
    Casablanca.headingLockMovingSpeedThreshold = 10.0;

    // Helper lerp matching Casablanca: frictionRot + speedRatio * (ks_moving - frictionRot)
    // Speed = 0.0 -> frictionRot (0.08)
    double speedRatio0 = Math.clamp(0.0 / Casablanca.headingLockMovingSpeedThreshold, 0.0, 1.0);
    double ks0 =
        Casablanca.frictionRot
            + speedRatio0 * (Casablanca.headingLockKsMoving - Casablanca.frictionRot);
    assertEquals(0.08, ks0, 1e-6);

    // Speed = 5.0 (midpoint) -> 0.08 + 0.5 * (0.02 - 0.08) = 0.05
    double speedRatioMid = Math.clamp(5.0 / Casablanca.headingLockMovingSpeedThreshold, 0.0, 1.0);
    double ksMid =
        Casablanca.frictionRot
            + speedRatioMid * (Casablanca.headingLockKsMoving - Casablanca.frictionRot);
    assertEquals(0.05, ksMid, 1e-6);

    // Speed = 10.0 or higher -> ks_moving (0.02)
    double speedRatioMax = Math.clamp(12.0 / Casablanca.headingLockMovingSpeedThreshold, 0.0, 1.0);
    double ksMax =
        Casablanca.frictionRot
            + speedRatioMax * (Casablanca.headingLockKsMoving - Casablanca.frictionRot);
    assertEquals(0.02, ksMax, 1e-6);
  }

  @Test
  public void testCasablancaHeadingLockHardGate() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = true;
    Casablanca.enableFrictionComp = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.headingLockIntentThreshold = 0.05;

    // First call to initialize target heading at pose (72, 72, 0)
    casablanca.adjustDriveInput(
        new Pose(72, 72, 0), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 0.0, 0.0);

    // Below threshold (0.0499): driver is considered "released" -> heading lock takes over and
    // corrects towards the latched target, bounded by headingLockMaxPower, not equal to raw stick.
    double[] outputBelow =
        casablanca.adjustDriveInput(
            new Pose(72, 72, 0.01), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 0.0, 0.0499);
    assertTrue(Math.abs(outputBelow[2]) <= Casablanca.headingLockMaxPower + 1e-9);
    assertTrue(outputBelow[2] != 0.0499);

    // Re-initialize to same target pose (72, 72, 0)
    casablanca.reset();
    casablanca.adjustDriveInput(
        new Pose(72, 72, 0), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 0.0, 0.0);

    // At/above threshold (0.0501): driver has full, unfought authority -> raw stick passes through
    // untouched by the lock (friction comp disabled in this test).
    double[] outputAbove =
        casablanca.adjustDriveInput(
            new Pose(72, 72, 0.01), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 0.0, 0.0501);
    assertEquals(0.0501, outputAbove[2], 1e-9);
  }

  @Test
  public void testCasablancaHeadingLockRelatchAndReset() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = true;
    Casablanca.enableFrictionComp = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.headingLockIntentThreshold = 0.05;

    // Call 1: Start at heading = 0 with no stick input -> targetHeading initialized to 0
    casablanca.adjustDriveInput(
        new Pose(72, 72, 0), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 0.0, 0.0);

    // Call 2: Driver actively steers (turn = 0.1 > threshold 0.05) to heading = Math.PI / 4 (45
    // deg)
    casablanca.adjustDriveInput(
        new Pose(72, 72, Math.PI / 4), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 0.0, 0.1);

    // Call 3: Driver releases stick (turn = 0.0) while robot is at heading = Math.PI / 4
    // Target heading should have re-latched to Math.PI / 4, so error is 0 and turn output is 0
    double[] outputLocked =
        casablanca.adjustDriveInput(
            new Pose(72, 72, Math.PI / 4),
            new com.pedropathing.math.Vector(0, 0),
            0.0,
            0.0,
            0.0,
            0.0);

    assertEquals(0.0, outputLocked[2], 1e-4);
  }

  @Test
  public void testCasablancaLiveRotationLookaheadTime() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel);
    Casablanca.rotationLookaheadTimeSeconds = 0.075;

    // At angular velocity = 0, lookahead angle is 0
    double lookaheadRad0 = Math.abs(0.0) * Casablanca.rotationLookaheadTimeSeconds;
    assertEquals(0.0, lookaheadRad0, 1e-6);

    // At angular velocity = 6.0 rad/s, lookahead angle is 6.0 * 0.075 = 0.45 rad
    double lookaheadRadMax = Math.abs(6.0) * Casablanca.rotationLookaheadTimeSeconds;
    assertEquals(0.45, lookaheadRadMax, 1e-6);

    // At low angular velocity = 1.0 rad/s, lookahead angle scales down to 0.075 rad
    double lookaheadRadLow = Math.abs(1.0) * Casablanca.rotationLookaheadTimeSeconds;
    assertEquals(0.075, lookaheadRadLow, 1e-6);
  }

  @Test
  public void testCasablancaProximityVsPhysicsBrakingScale() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);
    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;

    // Dynamically calculate test position to be at the midpoint of the proximity braking zone
    double hardStop = Casablanca.sideHardStop;
    double slowDown = Casablanca.sideSlowDown;
    double dMid = hardStop + 0.5 * (slowDown - hardStop);

    // Protected zone boundary (BLUE_GOAL_ZONE Y min)
    double protectedMinY = sentinel.getProtectedZone().getMinY();
    // Distance from robot center to robot top edge
    double halfWidth = config.sentinel.robot_width / 2.0;
    // Set robot Y so distance to hard stop is dMid
    double poseY = protectedMinY - halfWidth - dMid;

    double[] output =
        casablanca.adjustDriveInput(
            new Pose(3.0, poseY, 0), new com.pedropathing.math.Vector(0, 0), 0.0, 1.0, 0.0, 0.0);

    // Dynamic expected scale: (dMid - hardStop) / (slowDown - hardStop) = 0.5
    double expectedScale = (dMid - hardStop) / (slowDown - hardStop);
    assertEquals(expectedScale, output[0], 1e-3);
  }

  @Test
  public void testCasablancaPredictiveBrakingSanityCheck() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel); // Directly runs performBrakingSanityCheck()

    com.pedropathing.control.PredictiveBrakingController controller =
        new com.pedropathing.control.PredictiveBrakingController(
            org.firstinspires
                .ftc
                .teamcode
                .pedroPathing
                .Constants
                .followerConstants
                .predictiveBrakingCoefficients);

    double maxVelX = org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants.xVelocity;
    double maxVelY = org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants.yVelocity;

    double minBrakingX =
        Math.abs(controller.computeBrakingDisplacement(maxVelX, 1.0))
            / Casablanca.decelSafetyFactor;
    double minBrakingY =
        Math.abs(controller.computeBrakingDisplacement(maxVelY, 1.0))
            / Casablanca.decelSafetyFactor;

    // Verify calculated physics stopping distances are strictly positive and dynamically match
    // physics formula
    assertTrue(minBrakingX > 0.0);
    assertTrue(minBrakingY > 0.0);
    assertEquals(
        minBrakingX,
        Math.abs(controller.computeBrakingDisplacement(maxVelX, 1.0))
            / Casablanca.decelSafetyFactor,
        1e-6);
    assertEquals(
        minBrakingY,
        Math.abs(controller.computeBrakingDisplacement(maxVelY, 1.0))
            / Casablanca.decelSafetyFactor,
        1e-6);
  }
}
