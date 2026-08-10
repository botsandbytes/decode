package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.ballistics.ShotTimeTable;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.EndgameSpot;
import org.firstinspires.ftc.teamcode.records.Field;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.CalibrationRay;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;
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

    // Tests in this class exercise turret motion/aiming behavior, not the enabled flag itself
    // (see testTurretDisabledBehavior for that) -- config.turret.enabled defaults to false on a
    // freshly-built robot, so force it on here regardless of test execution order.
    config.turret.enabled = true;

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
  public void testTurretDirectionalKsFeedforward() {
    turret.setKs(0.08, 0.12);
    turret.setPIDF(0, 0, 0, 0);
    assertEquals(0.08, turret.getKsPositive(), 1e-6);
    assertEquals(0.12, turret.getKsNegative(), 1e-6);

    // Current angle at 0 deg, target at 10 deg (error > 0)
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(10.0);
    turret.updateTurret(new Pose(0, 0, 0));
    Mockito.verify(mockServo).setPower(Mockito.eq(0.08));

    // Current angle at 0 deg, target at -10 deg (error < 0)
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(-10.0);
    turret.updateTurret(new Pose(0, 0, 0));
    Mockito.verify(mockServo).setPower(Mockito.eq(-0.12));
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

  /**
   * Every pose autonomous shoots from has to be inside a launch zone.
   *
   * <p>Auto fires the same aimed shot TeleOp does, and that command ends itself the moment {@code
   * Sentinel.isLaunchAllowed} goes false. So a score pose that sits a few inches short of the zone
   * does not shoot badly — it does not shoot at all, silently, for a whole match. That failure has
   * no signature on the driver station beyond a zero, which is exactly why it belongs in a test.
   *
   * <p>This checks the configured poses. What the robot actually reaches after following a path is
   * reported live by the "Launch Legal" telemetry line in {@code AllianceAutoBase}.
   */
  @Test
  public void testAutoScorePosesAreInsideLaunchZone() {
    for (Alliance alliance : Alliance.values()) {
      // isLaunchAllowed checks both zones and is alliance-independent; the alliance here selects
      // which set of (mirrored) poses gets checked.
      Sentinel sentinel = new Sentinel(alliance);
      String side = alliance == Alliance.RED ? "red" : "blue";

      config.NormalAuto normal =
          ConfigLoader.loadMerged(config.NormalAuto.class, "auto_poses.normal." + side, "auto");
      assertTrue(
          side + " normal auto score pose is outside every launch zone",
          sentinel.isLaunchAllowed(normal.score));
      assertTrue(
          side + " normal auto final_score pose is outside every launch zone",
          sentinel.isLaunchAllowed(normal.finalScore));

      config.OppositeAuto opposite =
          ConfigLoader.loadMerged(config.OppositeAuto.class, "auto_poses.opposite." + side, "auto");
      assertTrue(
          side + " opposite auto score pose is outside every launch zone",
          sentinel.isLaunchAllowed(opposite.score));
    }
  }

  /**
   * The endgame parking spot: committed to one side of the launch zone boundary, on our own half,
   * and the nearest such point.
   *
   * <p>Auto has about two seconds to execute whatever this returns, so a target that is subtly
   * wrong is not recoverable at runtime — a spot that still straddles the line leaves the robot in
   * exactly the state the whole feature exists to avoid, and one across the midline trades a minor
   * foul for a worse one. All of it is pure geometry, so all of it is checkable here.
   *
   * <p>Checked from each alliance's own score pose, which by {@link
   * #testAutoScorePosesAreInsideLaunchZone} overlaps a launch zone — the exact situation the timer
   * fires in.
   */
  @Test
  public void testEndgameSpotIsCommittedNearAndOnOurSide() {
    double margin = config.auto.endgame.exit_clearance;
    double halfDiagonal = config.sentinel.robot_width * Math.sqrt(2.0) / 2.0;

    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      String side = alliance == Alliance.RED ? "red" : "blue";
      boolean ownSideIsHighX = Field.getGoalX(alliance) > 72.0;

      config.OppositeAuto opposite =
          ConfigLoader.loadMerged(config.OppositeAuto.class, "auto_poses.opposite." + side, "auto");
      Pose score = opposite.score;
      EndgameSpot spot = sentinel.nearestEndgameSpot(score, margin);

      assertNotNull(side + " has no committed endgame spot from its score pose", spot);
      Pose parked = spot.pose();

      // Committed at every heading, and matching what the spot claims about itself.
      Sentinel.ZoneStanding expected =
          spot.insideLaunchZone() ? Sentinel.ZoneStanding.INSIDE : Sentinel.ZoneStanding.OUTSIDE;
      for (double heading = 0; heading < 2 * Math.PI; heading += Math.PI / 8) {
        Pose rotated = new Pose(parked.getX(), parked.getY(), heading);
        assertEquals(
            side + " endgame spot is not committed at heading " + heading,
            expected,
            sentinel.zoneStanding(rotated, margin));

        for (Coordinate corner : sentinel.calculateRobotFootprint(rotated)) {
          assertTrue(
              side + " endgame footprint crosses the midline at heading " + heading,
              ownSideIsHighX ? corner.x >= 72.0 : corner.x <= 72.0);
          assertTrue(
              side + " endgame footprint leaves the field", corner.x >= 0 && corner.x <= 144);
          assertTrue(
              side + " endgame footprint leaves the field", corner.y >= 0 && corner.y <= 144);
        }
      }

      // "Shortest path": nothing meaningfully closer is committed, in either direction — the search
      // has to consider going deeper into the zone as well as out of it. The slack is what the
      // heading-blind model deliberately gives away: it plans against the footprint's circumscribed
      // circle, while the check below samples 16 discrete headings of the real square.
      double spotDistance = score.distanceFrom(parked);
      double slack = margin + 0.5;
      assertTrue(side + " endgame move is implausibly long", spotDistance < 3 * halfDiagonal);
      for (double angle = 0; angle < 2 * Math.PI; angle += Math.PI / 24) {
        for (double r = 0.5; r < spotDistance - slack; r += 0.5) {
          double cx = score.getX() + r * Math.cos(angle);
          double cy = score.getY() + r * Math.sin(angle);
          boolean committedAtEveryHeading = true;
          for (double h = 0; h < 2 * Math.PI && committedAtEveryHeading; h += Math.PI / 8) {
            committedAtEveryHeading =
                sentinel.zoneStanding(new Pose(cx, cy, h), margin)
                    != Sentinel.ZoneStanding.ON_BOUNDARY;
          }
          boolean onOurSide =
              ownSideIsHighX ? cx - halfDiagonal >= 72.0 : cx + halfDiagonal <= 72.0;
          assertFalse(
              String.format(
                  "%s has a committed point closer than the spot: (%.1f, %.1f)", side, cx, cy),
              committedAtEveryHeading && onOurSide);
        }
      }

      // A robot already committed is told to stay where it is.
      EndgameSpot stay = sentinel.nearestEndgameSpot(parked, margin);
      assertEquals(parked.getX(), stay.pose().getX(), 1e-9);
      assertEquals(parked.getY(), stay.pose().getY(), 1e-9);
    }
  }

  /**
   * The small far triangle cannot hold this robot, which is why "get fully inside a zone" is only
   * ever an option in the big one.
   *
   * <p>{@link Sentinel#nearestEndgameSpot} relies on this rather than special-casing it: eroding
   * the zones by the footprint deletes the small triangle entirely. If the robot ever shrinks (or
   * the zone grows) enough for it to fit, this test fails and the comment there stops being true —
   * which is the point of asserting it.
   */
  @Test
  public void testSmallLaunchZoneCannotContainTheRobot() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    double margin = config.auto.endgame.exit_clearance;

    // The small zone spans x in [48, 96], y in [0, 24]. Sweep it densely at several headings.
    for (double x = 48; x <= 96; x += 1.0) {
      for (double y = 0; y <= 24; y += 1.0) {
        for (double h = 0; h < Math.PI / 2; h += Math.PI / 16) {
          assertFalse(
              String.format("robot reported fully inside the small zone at (%.0f, %.0f)", x, y),
              sentinel.zoneStanding(new Pose(x, y, h), margin) == Sentinel.ZoneStanding.INSIDE);
        }
      }
    }
  }

  /**
   * The ball counter behind the Shot Timing Tuner.
   *
   * <p>Everything that makes this detector correct is a threshold, and every one of them fails in a
   * way that still looks like a plausible answer: no refractory turns one ball into four, a missing
   * latch re-fires the same dip the instant the refractory expires, and a baseline that averages
   * during a dip stops counting halfway through a feed. None of that is visible from a driver
   * station reading "6 balls" — you would simply tune auto's window against a number that was never
   * real.
   */
  @Test
  public void testFlywheelDipDetectorCountsOneEventPerBall() {
    // 5% below reference triggers, 5% rebound off the trough re-arms, refractory 120 ms.
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.03, 120);

    // Settle a baseline at 1000 ticks/s.
    for (int i = 0; i < 100; i++) {
      assertFalse(detector.update(1000.0, i * 10L));
    }
    assertEquals(1000.0, detector.getBaselineVelocity(), 1e-6);

    // One ball: a dip held over several loops fires exactly once, on the leading edge.
    assertTrue(detector.update(920.0, 1000L));
    assertFalse(detector.update(900.0, 1010L));
    assertFalse(detector.update(910.0, 1020L));
    assertEquals(1, detector.getEventCount());
    assertEquals(0.10, detector.getDeepestDipFraction(), 1e-9);

    // Recovery above the re-arm line, then a second ball past the refractory window.
    assertFalse(detector.update(1000.0, 1030L));
    assertTrue(detector.update(930.0, 1400L));
    assertEquals(2, detector.getEventCount());

    // A dip that re-crosses inside the refractory window is the same ball's ragged floor.
    assertFalse(detector.update(1000.0, 1410L));
    assertFalse(detector.update(930.0, 1450L));
    assertEquals(2, detector.getEventCount());

    // Noise that never reaches the trigger is not a ball.
    assertFalse(detector.update(1000.0, 2000L));
    assertFalse(detector.update(970.0, 2010L));
    assertEquals(2, detector.getEventCount());

    // Sign is ignored, as everywhere else that reads flywheel velocity.
    assertFalse(detector.update(-1000.0, 2100L));
    assertEquals(2, detector.getEventCount());
  }

  /**
   * Replays the shape of a real two-ball trace off this robot, where the second ball arrives long
   * before the wheel has recovered from the first.
   *
   * <p>The numbers are from {@code shot_timing_log.csv}: idling at 1020 ticks/s, ball one drops it
   * to a 740 trough, the controller claws back to 980 over about 1.4 s, and ball two lands on top
   * of that partial recovery. An earlier version ended a dip only when the wheel returned within 2%
   * of its pre-ball reference — which took 2.2 s, so it stayed latched through ball two and
   * reported one ball for the magazine. Ending the dip on a rebound off the trough is what makes
   * the second one visible.
   */
  @Test
  public void testFlywheelDipDetectorCountsBallLandingBeforeFullRecovery() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.03, 120);

    long t = 0;
    for (int i = 0; i < 20; i++, t += 10) {
      assertFalse(detector.update(1020.0, t));
    }

    // Ball one: 1020 -> 740 trough.
    assertTrue("ball one missed", detector.update(920.0, t));
    t += 10;
    for (double v : new double[] {860, 800, 760, 740}) {
      detector.update(v, t);
      t += 10;
    }
    assertEquals(1, detector.getEventCount());

    // Partial recovery — nowhere near the 1020 it started from.
    for (double v : new double[] {760, 800, 840, 860, 880, 920, 940, 980}) {
      detector.update(v, t);
      t += 10;
    }
    assertEquals("recovery must not be counted as a ball", 1, detector.getEventCount());

    // Ball two lands on that partial recovery.
    assertTrue("ball two missed — detector latched in ball one's dip", detector.update(800.0, t));
    assertEquals(2, detector.getEventCount());
  }

  /**
   * A feed that never lets the wheel back up to its setpoint still has to count every ball.
   *
   * <p>This is the case that rules out measuring dips against the commanded target: during a
   * sustained feed the flywheel sits well below setpoint the whole time — that sag is exactly why
   * {@code feed_release_threshold} is 0.80 while the arm window is 0.97 — so a target-relative
   * threshold sees one enormous dip and reports a single ball for the entire burst.
   */
  @Test
  public void testFlywheelDipDetectorCountsBallsWhileWheelStaysSagged() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.2, 100);

    // Wheel has settled at 850 ticks/s under a continuous feed — 15% under a 1000 setpoint.
    for (int i = 0; i < 100; i++) {
      detector.update(850.0, i * 10L);
    }
    assertEquals(850.0, detector.getBaselineVelocity(), 1e-3);

    long t = 2000L;
    for (int ball = 0; ball < 4; ball++) {
      assertTrue("ball " + ball + " missed", detector.update(780.0, t));
      assertEquals(ball + 1, detector.getEventCount());
      // Recover to the local baseline, not to the setpoint.
      for (int i = 0; i < 12; i++) {
        detector.update(850.0, t + 10L + i * 10L);
      }
      t += 300L;
    }
    assertEquals(4, detector.getEventCount());
  }

  /**
   * A wheel pinned at the bottom of a dip is one ball, however long it stays there.
   *
   * <p>The reference must not average downward toward it: if it tracks the ball down, the wheel
   * eventually looks "recovered" without having moved, and the same stuck ball counts again.
   */
  /**
   * The flywheel coasting down must not read as a ball.
   *
   * <p>This is the failure that corrupted a whole calibration run. After the last ball the wheel
   * sits above its setpoint and bleeds back down over more than a second, one encoder quantum per
   * loop. Because the reference follows the wheel up instantly and down slowly, it hangs above the
   * decay — and with the trigger set at 5% it eventually crossed, reporting a third ball in all 11
   * recorded shots and cutting each measurement short before the real one landed. Real balls
   * measured 12.2%-26.4% deep against a 5.4% worst case for the coast, so the trigger belongs
   * between them.
   */
  @Test
  public void testFlywheelDipDetectorIgnoresCoastDown() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.09, 0.05, 0.03, 120);

    // Overshoot peak, then the real measured bleed-down: 1280 -> 1180 in 20-tick steps.
    long t = 0;
    for (int i = 0; i < 20; i++, t += 20) {
      detector.update(1280.0, t);
    }
    for (double v :
        new double[] {
          1280, 1260, 1280, 1260, 1240, 1260, 1240, 1220, 1220, 1200, 1200, 1200, 1180, 1180
        }) {
      assertFalse("coast-down at " + v + " must not count as a ball", detector.update(v, t));
      t += 20;
    }
    assertEquals(0, detector.getEventCount());

    // A real ball off that same decayed wheel is still caught: 1180 -> 980 is 17%.
    assertTrue("a real ball after the coast must still register", detector.update(980.0, t));
    assertEquals(1, detector.getEventCount());
  }

  @Test
  public void testFlywheelDipDetectorHoldsOneEventWhilePinnedInDip() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.2, 50);
    for (int i = 0; i < 100; i++) {
      detector.update(1000.0, i * 10L);
    }

    double baselineBefore = detector.getBaselineVelocity();
    for (int i = 0; i < 40; i++) {
      detector.update(900.0, 2000L + i * 10L);
    }
    assertEquals(baselineBefore, detector.getBaselineVelocity(), 1e-9);
    assertEquals(1, detector.getEventCount());
  }

  /**
   * The calibration ray must stay inside the legal drive box and always face the goal.
   *
   * <p>{@code maxDistance} is what stops an unreachable endpoint being clamped onto a different
   * bearing and recorded under the distance that was asked for rather than the one actually shot.
   */
  @Test
  public void testCalibrationRayStaysInBoxAndFacesGoal() {
    double gx = Field.getBlueGoalX();
    double gy = Field.getBlueGoalY();

    double maxDistance = CalibrationRay.maxDistance(gx, gy);
    assertTrue("ray should reach well past the shot table's near end", maxDistance > 50.0);

    Pose atLimit = CalibrationRay.waypoint(gx, gy, maxDistance);
    assertTrue(
        "ray must not leave the box in x", atLimit.getX() <= CalibrationRay.MAX_TARGET_X + 1e-6);
    assertTrue(
        "ray must not leave the box in y", atLimit.getY() >= CalibrationRay.MIN_TARGET_Y - 1e-6);

    for (double distance : new double[] {40.0, 72.0, 120.0}) {
      Pose waypoint = CalibrationRay.waypoint(gx, gy, distance);
      // The waypoint sits exactly `distance` from the goal...
      assertEquals(distance, Math.hypot(gx - waypoint.getX(), gy - waypoint.getY()), 1e-6);
      // ...and points straight back at it, which is what makes the turret's job the same at every
      // endpoint and the recorded distance the only thing that varies.
      assertEquals(
          Math.atan2(gy - waypoint.getY(), gx - waypoint.getX()), waypoint.getHeading(), 1e-9);
    }
  }

  /**
   * The measured shoot-window lookup that bounds every autonomous scoring cycle.
   *
   * <p>Clamping outside the measured range is the deliberate choice, and the opposite of what
   * {@link org.firstinspires.ftc.teamcode.ballistics.ShotTable} does with an uncalibrated distance:
   * a window that is too long only costs autonomous time, while one that is too short leaves balls
   * in the robot with no indication that it happened.
   */
  @Test
  public void testShotTimeTableInterpolatesAndClamps() {
    ShotTimeTable table = ShotTimeTable.fromPoints(new double[] {950.0, 5000, 1100.0, 3000});
    assertEquals(2, table.size());

    // Exact rows.
    assertEquals(5000, table.lookupMs(950.0));
    assertEquals(3000, table.lookupMs(1100.0));

    // Linear between them.
    assertEquals(4000, table.lookupMs(1025.0));

    // Clamped, not extrapolated: a shot fired slower than anything measured does not get faster.
    assertEquals(5000, table.lookupMs(800.0));
    assertEquals(3000, table.lookupMs(2000.0));

    // A non-finite RPM must still yield a usable window rather than an exception, since the
    // alternative is a scoring cycle with no bound on it at all.
    assertEquals(5000, table.lookupMs(Double.NaN));
  }

  /** An absent or malformed table must never leave a scoring cycle with no bound on it. */
  @Test
  public void testShotTimeTableFallsBackWhenUnusable() {
    assertNull(ShotTimeTable.fromPoints(new double[] {}));
    assertNull(ShotTimeTable.fromPoints(null));

    // windowMsFor never throws and never returns zero, whatever it is handed — a zero window feeds
    // nothing, and a missing one never gives the chassis back to the next path.
    for (double rpm : new double[] {Double.NaN, -50.0, 0.0, 1000.0, 1e9}) {
      assertTrue("window must be positive at " + rpm, ShotTimeTable.windowMsFor(rpm) > 0);
    }

    try {
      ShotTimeTable.fromPoints(new double[] {950.0, 5000, 1100.0});
      org.junit.Assert.fail("an odd number of values is not rpm,window pairs");
    } catch (IllegalStateException expected) {
      assertTrue(expected.getMessage().contains("pairs"));
    }

    try {
      ShotTimeTable.fromPoints(new double[] {950.0, 5000, 900.0, 3000});
      org.junit.Assert.fail("rpms going backwards must be rejected, not silently sorted");
    } catch (IllegalStateException expected) {
      assertTrue(expected.getMessage().contains("strictly increase"));
    }
  }

  /**
   * The shipped table has to actually cover the RPM the autos actually shoot at.
   *
   * <p>Outside the measured range {@link ShotTimeTable} clamps, and clamping across a long gap is
   * how a scoring cycle silently gets the wrong budget. The curve runs backwards — low-RPM shots
   * take longer than high-RPM ones — so a fast shot clamped onto a slow row is handed seconds it
   * does not need, and a slow shot clamped onto a fast row is cut off with balls still in the
   * robot.
   *
   * <p>The RPM checked here is what {@link org.firstinspires.ftc.teamcode.ballistics.ShotTable}
   * commands <i>right now</i> for each score pose's distance — not the RPM the shot-time table was
   * originally measured against — since that current mapping is what autonomous will actually fire
   * at. If a {@code shooter.shot_table} recalibration moves a score distance onto an RPM the timing
   * table no longer covers, this is the test that catches it.
   */
  @Test
  public void testConfiguredShotTimeTableCoversAutoScoreRpm() {
    ShotTimeTable table = ShotTimeTable.fromConfig();
    assertTrue(
        "auto.shot_time_points is empty — autos fall back to the flat window", table != null);
    ShotTable shotTable = ShotTable.fromConfig();

    double allowedClampRpm = 40.0;

    config.NormalAuto normal =
        ConfigLoader.loadMerged(config.NormalAuto.class, "auto_poses.normal.blue", "auto");
    config.OppositeAuto opposite =
        ConfigLoader.loadMerged(config.OppositeAuto.class, "auto_poses.opposite.blue", "auto");

    for (Object[] which : new Object[][] {{"normal", normal.score}, {"opposite", opposite.score}}) {
      Pose score = (Pose) which[1];
      double distance =
          Math.hypot(Field.getBlueGoalX() - score.getX(), Field.getBlueGoalY() - score.getY());
      double rpm = shotTable.lookup(distance).rpm();
      assertTrue(
          String.format(
              "%s auto scores at %.0f in (%.0f rpm), more than %.0f rpm outside the measured"
                  + " %.0f-%.0f rpm shot-time table — re-run the Shot Timing Tuner to cover it",
              which[0], distance, rpm, allowedClampRpm, table.minRpm(), table.maxRpm()),
          rpm >= table.minRpm() - allowedClampRpm && rpm <= table.maxRpm() + allowedClampRpm);
    }
  }

  @Test
  public void testCasablancaSymmetryAndCollisionMath() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    // Disable heading lock, input smoothing, and friction compensation for predictable math testing
    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;
    Casablanca.fieldCentric = false;

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
  public void testCasablancaGoalHeadingLock() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;
    Casablanca.fieldCentric = false;
    // The driver's face-the-goal button is an explicit request, so it must engage even with the
    // automatic "hold the last heading you were left at" lock switched off in config.
    Casablanca.enableHeadingLock = false;

    // Field centre, clear of both goal zones, so the rotation-safety gate cannot zero the turn.
    Pose pose = new Pose(72, 72, 0.0);
    com.pedropathing.math.Vector rest = new com.pedropathing.math.Vector(0, 0);

    // Lock off: no turn stick means no turn, because the automatic lock is disabled.
    assertEquals(0.0, casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2], 1e-9);
    assertFalse(casablanca.isGoalHeadingLockActive());

    // Goal 90 deg to the robot's left. The bearing comes from the raw goal position, matching what
    // TeleOp publishes via Turret.alignPose -- no velocity lead, so a stationary robot and a
    // moving one at the same place get the same target.
    double goalBearing = Turret.alignPose(pose.getX(), pose.getY(), 72, 100).getHeading();
    assertEquals(Math.PI / 2, goalBearing, 1e-6);

    casablanca.setGoalHeadingLock(goalBearing, true);
    assertTrue(casablanca.isGoalHeadingLockActive());

    // Corrects on the very first loop: unlike the latching lock, there is no wait for the robot to
    // stop turning before a target exists.
    double turnToGoal = casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2];
    assertTrue(turnToGoal > 0);
    assertTrue(Math.abs(turnToGoal) <= Casablanca.headingLockMaxPower + 1e-9);

    // Same target while translating: the lock is a heading-only override, so a moving robot still
    // gets a correction toward the unmodified goal bearing rather than a lead-compensated one.
    double turnWhileMoving =
        casablanca
            .adjustDriveInput(pose, new com.pedropathing.math.Vector(30, 0), 0.0, 0.0, 1.0, 0.0)[2];
    assertTrue(turnWhileMoving > 0);

    // ShotController's armed aim outranks it: both channels active, the solved azimuth wins.
    casablanca.setArmedAimTarget(-Math.PI / 2, true);
    double turnArmed = casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2];
    assertTrue(turnArmed < 0);

    // Clearing armed aim (stopShot) falls back to the driver's lock, not to no lock at all.
    casablanca.setArmedAimTarget(0.0, false);
    assertTrue(casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2] > 0);

    // Toggling the lock off hands the turn axis back to the driver.
    casablanca.setGoalHeadingLock(goalBearing, false);
    assertFalse(casablanca.isGoalHeadingLockActive());
    assertEquals(0.0, casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2], 1e-9);

    Casablanca.enableHeadingLock = true;
  }

  @Test
  public void testTurretHoldAngleIsChassisRelativeAndHeadingInvariant() {
    // The analog encoder reads the turret's angle relative to the chassis directly (unlike the
    // old IMU, which read an absolute field heading). Physically offset the turret from the
    // chassis and hold it there.
    //
    // Derived from configured travel rather than hardcoded: setHoldAngle() clamps to
    // [min_angle, max_angle], so a literal angle outside the currently calibrated travel would
    // clamp the target away from the physical angle and fail this assert for reasons that have
    // nothing to do with heading invariance.
    double offsetAngle = config.turret.travel.max_angle * 0.5;
    setPhysicalTurretAngle(offsetAngle);
    turret.setHoldAngle(offsetAngle);
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
    setPhysicalTurretAngle(config.turret.travel.max_angle + 1e-3);
    turret.setTurretPowerRaw(0.4);
    Mockito.verify(mockServo).setPower(0.0);

    // Driving back the other way off the limit must still be allowed.
    turret.setTurretPowerRaw(-0.4);
    Mockito.verify(mockServo).setPower(-0.4);
  }

  @Test
  public void testManualModeAllowsFullTravelWhenCentered() {
    // Centered turret must retain its full +/-45 deg range in MANUAL mode.
    setPhysicalTurretAngle(0.0);
    turret.setTurretPowerRaw(0.4);
    Mockito.verify(mockServo).setPower(0.4);

    Mockito.reset(mockServo);
    turret.setTurretPowerRaw(-0.4);
    Mockito.verify(mockServo).setPower(-0.4);
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
    Casablanca.fieldCentric = false;
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
    Casablanca.fieldCentric = false;
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
    Casablanca.fieldCentric = false;

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
  public void testCasablancaFieldCentricIsHeadingInvariant() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;
    Casablanca.fieldCentric = true;
    Casablanca.fieldCentricOffsetRad = Math.toRadians(90.0);

    // The whole contract of field-centric drive: one stick direction maps to one field direction
    // no matter which way the robot is pointing. Output is {fieldY, fieldX, turn}.
    double[] reference = null;
    for (double headingDeg : new double[] {0.0, 90.0, 180.0, 270.0, 37.0}) {
      casablanca.reset();
      double[] out =
          casablanca.adjustDriveInput(
              new Pose(72, 72, Math.toRadians(headingDeg)),
              new com.pedropathing.math.Vector(0, 0),
              0.0,
              0.0,
              1.0,
              0.0);
      if (reference == null) {
        reference = out;
      } else {
        assertEquals(reference[0], out[0], 1e-6);
        assertEquals(reference[1], out[1], 1e-6);
      }
    }

    // The 90 degree offset means a fully-forward stick drives along field +Y -- away from the
    // driver wall at y = 0 -- rather than along field +X.
    assertEquals(0.0, reference[1], 1e-6);
    assertTrue(reference[0] > 0.0);

    // Teeth for the assertions above: the same stick IS heading-dependent with field-centric off,
    // so heading-invariance is the transform doing work and not a degenerate all-zero output.
    Casablanca.fieldCentric = false;
    casablanca.reset();
    double[] robotAt0 =
        casablanca.adjustDriveInput(
            new Pose(72, 72, 0.0), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 1.0, 0.0);
    casablanca.reset();
    double[] robotAt90 =
        casablanca.adjustDriveInput(
            new Pose(72, 72, Math.PI / 2),
            new com.pedropathing.math.Vector(0, 0),
            0.0,
            0.0,
            1.0,
            0.0);
    assertTrue(Math.abs(robotAt0[0] - robotAt90[0]) > 0.5);
  }

  @Test
  public void testCasablancaRefusesToDriveOnNonFinitePose() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;

    // A NaN heading is a dead localizer. Vector.rotateVector(NaN) produces NaN components even for
    // a zero-magnitude vector, and every JTS Envelope comparison against NaN is false, so the zone
    // protections would silently disengage while NaN reached the drivetrain. Refuse instead.
    for (boolean fieldCentric : new boolean[] {true, false}) {
      Casablanca.fieldCentric = fieldCentric;
      casablanca.reset();
      double[] out =
          casablanca.adjustDriveInput(
              new Pose(72, 72, Double.NaN),
              new com.pedropathing.math.Vector(0, 0),
              0.0,
              0.5,
              1.0,
              0.5);
      assertEquals(0.0, out[0], 1e-9);
      assertEquals(0.0, out[1], 1e-9);
      assertEquals(0.0, out[2], 1e-9);
      assertTrue(casablanca.wasPoseUntrusted());
    }

    // And it clears again once the localizer recovers.
    Casablanca.fieldCentric = false;
    casablanca.reset();
    casablanca.adjustDriveInput(
        new Pose(72, 72, 0.0), new com.pedropathing.math.Vector(0, 0), 0.0, 0.0, 1.0, 0.0);
    assertFalse(casablanca.wasPoseUntrusted());
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

  @Test
  public void testShotControllerStopShotResetsHoldAngleToZero() {
    org.firstinspires.ftc.teamcode.robot.Shooter mockShooter =
        Mockito.mock(org.firstinspires.ftc.teamcode.robot.Shooter.class);
    org.firstinspires.ftc.teamcode.robot.Intake mockIntake =
        Mockito.mock(org.firstinspires.ftc.teamcode.robot.Intake.class);
    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);

    // Robot has non-zero heading (e.g. 50 degrees)
    Pose nonZeroHeadingPose = new Pose(72, 72, Math.toRadians(50.0));
    org.firstinspires.ftc.teamcode.robot.ShotController shotController =
        new org.firstinspires.ftc.teamcode.robot.ShotController(
            mockShooter,
            turret,
            mockIntake,
            () -> nonZeroHeadingPose,
            () -> new Pose(0, 0, 0),
            null,
            Alliance.BLUE,
            mockTelemetry);

    shotController.startShot(0.8, true);
    shotController.stopShot();

    // Verify hold angle is set to 0.0 chassis-relative center, NOT 50.0 degrees
    assertEquals(0.0, turret.getHoldAngle(), 1e-6);
    assertEquals(Turret.AimMode.HOLD, turret.getAimMode());
  }

  @Test
  public void testTurretDisabledBehavior() {
    assertTrue(turret.isEnabled());

    // 1. Flip via config flag
    config.turret.enabled = false;
    assertFalse(turret.isEnabled());
    assertTrue(turret.isAimed(new Pose(0, 0, 0)));
    assertTrue(turret.isTurnDone());

    turret.setTurretPowerRaw(0.5);
    assertEquals("TURRET DISABLED", turret.getPowerBlockedReason());
    Mockito.verify(mockServo, Mockito.never()).setPower(0.5);

    config.turret.enabled = true;
    assertTrue(turret.isEnabled());

    // 2. Flip dynamically via turret.setEnabled(false)
    turret.setEnabled(false);
    assertFalse(turret.isEnabled());

    turret.setEnabled(true);
    assertTrue(turret.isEnabled());
  }

  @Test
  public void testShotControllerSetInitialSolution() {
    HardwareMap hardwareMap = Mockito.mock(HardwareMap.class);
    DcMotorEx mockShooter1 = Mockito.mock(DcMotorEx.class);
    DcMotorEx mockShooter2 = Mockito.mock(DcMotorEx.class);
    com.qualcomm.robotcore.hardware.Servo mockHood =
        Mockito.mock(com.qualcomm.robotcore.hardware.Servo.class);
    com.qualcomm.robotcore.hardware.VoltageSensor mockVoltage =
        Mockito.mock(com.qualcomm.robotcore.hardware.VoltageSensor.class);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "shooter")).thenReturn(mockShooter1);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "shooter2")).thenReturn(mockShooter2);
    Mockito.when(hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "hood"))
        .thenReturn(mockHood);
    java.util.List<com.qualcomm.robotcore.hardware.VoltageSensor> sensors =
        java.util.Collections.singletonList(mockVoltage);
    hardwareMap.voltageSensor =
        Mockito.mock(com.qualcomm.robotcore.hardware.HardwareMap.DeviceMapping.class);
    Mockito.when(hardwareMap.voltageSensor.iterator()).thenAnswer(inv -> sensors.iterator());

    org.firstinspires.ftc.teamcode.robot.Shooter shooter =
        new org.firstinspires.ftc.teamcode.robot.Shooter(hardwareMap);
    org.firstinspires.ftc.teamcode.robot.ShotController shotController =
        new org.firstinspires.ftc.teamcode.robot.ShotController(
            shooter, null, null, null, null, null, Alliance.RED, null);

    org.firstinspires.ftc.teamcode.records.ShotSolution primedSolution =
        new org.firstinspires.ftc.teamcode.records.ShotSolution(
            0.65, 1150.0, 220.0, 0.0, 0.4, 0.0, 65.0, true, "Valid");

    shotController.setInitialSolution(primedSolution);

    assertEquals(primedSolution, shotController.getLastSolution());
    assertEquals(0.65, shooter.getTargetHoodPosition(), 1e-6);

    shotController.shutdown();
  }
}
