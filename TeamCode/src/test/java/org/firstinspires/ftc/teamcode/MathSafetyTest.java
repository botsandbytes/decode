package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.Before;
import org.junit.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.mockito.Mockito;

public class MathSafetyTest {

  private Turret turret;
  private HardwareMap hardwareMap;

  @Before
  public void setUp() {
    // Mock hardware dependencies for Turret initialization
    hardwareMap = Mockito.mock(HardwareMap.class);
    CRServo mockServo = Mockito.mock(CRServo.class);
    IMU mockIMU = Mockito.mock(IMU.class);
    DcMotorEx mockMotor = Mockito.mock(DcMotorEx.class);

    Mockito.when(hardwareMap.get(CRServo.class, "turn")).thenReturn(mockServo);
    Mockito.when(hardwareMap.get(IMU.class, "turnImu")).thenReturn(mockIMU);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftBack")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightBack")).thenReturn(mockMotor);

    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);
    turret = new Turret(hardwareMap, mockTelemetry);
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
    config.turret.turn.offset_const = 90.0;
    config.turret.turn.limit_const = 180.0;

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
    // Verify Sentinel goal zones bounding box calculations (loaded from config.yaml)
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Envelope blueZone = sentinel.getBlueGoalZone();
    assertEquals(0.0, blueZone.getMinX(), 1e-6);
    assertEquals(69.0, blueZone.getMinY(), 1e-6);
    assertEquals(6.0, blueZone.getMaxX(), 1e-6);
    assertEquals(75.0, blueZone.getMaxY(), 1e-6);

    Envelope redZone = sentinel.getRedGoalZone();
    assertEquals(138.0, redZone.getMinX(), 1e-6);
    assertEquals(69.0, redZone.getMinY(), 1e-6);
    assertEquals(144.0, redZone.getMaxX(), 1e-6);
    assertEquals(75.0, redZone.getMaxY(), 1e-6);
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
    // Corner 0: (10 - 9, 10 - 9) = (1, 1)
    assertEquals(1.0, footprintZero[0].x, 1e-4);
    assertEquals(1.0, footprintZero[0].y, 1e-4);
    // Corner 1: (10 + 9, 10 - 9) = (19, 1)
    assertEquals(19.0, footprintZero[1].x, 1e-4);
    assertEquals(1.0, footprintZero[1].y, 1e-4);

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
    // Place robot near the blue goal zone at (15.5, 72) with heading = 0.
    Pose poseRotationSafeCheck = new Pose(15.5, 72, 0);
    assertFalse(sentinel.isRotationSafe(poseRotationSafeCheck, 1.0, 0.5));
  }

  @Test
  public void testCasablancaSymmetryAndCollisionMath() {
    Sentinel sentinel =
        new Sentinel(
            Alliance.RED); // protected opponent zone is BLUE_GOAL_ZONE: left=0, top=69, right=6,
    // bottom=75
    Casablanca casablanca = new Casablanca(sentinel);

    // Disable heading lock, input smoothing, and friction compensation for predictable math testing
    casablanca.enableHeadingLock = false;
    casablanca.enableInputSmoothing = false;
    casablanca.enableFrictionComp = false;

    // --- CASE 1: Depth (X-axis) Protection Check ---
    // Place robot at x = 14.5 (left edge is 14.5 - 9 = 5.5, violating blue goal's right edge at
    // 6.0), y = 72.
    // Try to drive forward (which goes into the blue goal).
    double[] outputX =
        casablanca.adjustDriveInput(
            new Pose(14.5, 72, 0), new com.pedropathing.math.Vector(0, 0), 0.0, -1.0, 0.0);
    // Adjusted forward (index 1) should be scaled down significantly (or 0)
    assertTrue(Math.abs(outputX[1]) < 0.2);

    // --- CASE 2: Side (Y-axis) Protection Check ---
    // Place robot at x = 3.0, y = 60.0 (top edge is 60 + 9 = 69.0, touching blue goal's bottom edge
    // at 69.0).
    // Try to strafe positive Y (towards the blue goal).
    double[] outputY =
        casablanca.adjustDriveInput(
            new Pose(3.0, 60.0, 0), new com.pedropathing.math.Vector(0, 0), 1.0, 0.0, 0.0);
    // Adjusted strafe (index 0) should be scaled down significantly (or 0)
    assertTrue(Math.abs(outputY[0]) < 0.2);
  }
}
