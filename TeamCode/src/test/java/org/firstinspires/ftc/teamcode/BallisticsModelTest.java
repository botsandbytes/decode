package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.ballistics.BallisticsModel;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.junit.Before;
import org.junit.Test;

public class BallisticsModelTest {

  private BallisticsParameters params;

  @Before
  public void setUp() {
    params =
        new BallisticsParameters(
            360.0, // v0 = 360 in/s (30 ft/s)
            0.0012, // k drag
            0.015, // L Magnus lift
            15.0, // launch height inches
            45.0, // goal height inches
            386.088, // g in/s2
            15.0, // min hood deg
            65.0, // max hood deg
            0.15, // min servo pos
            0.85, // max servo pos
            60.0, // max speed ips
            30.0, // min dist
            160.0, // max dist
            1.0, // lead bias gain
            1000.0, // v0 reference rpm
            1000.0, // preferred shot rpm
            1450.0, // max shot rpm
            0.12, // v0 margin fraction
            0.004, // flight time sec per inch
            10.0 // lead min speed ips
            );
  }

  /**
   * The real hood linkage is reversed: servo 0.0 is the steepest arc, so the servo position for the
   * minimum angle is numerically the LARGER of the pair. An earlier unsigned span guard collapsed
   * that negative span to a divide-by-1e-6 and reported every steep angle as its shallow mirror,
   * which silently corrupted a whole calibration run's recorded hood angles.
   */
  @Test
  public void testReversedHoodLinkageMapping() {
    BallisticsParameters reversed =
        new BallisticsParameters(
            360.0, 0.0012, 0.015, 15.0, 45.0, 386.088, 15.0, // min hood deg -> flattest
            65.0, // max hood deg -> steepest
            0.85, // servo AT MIN ANGLE (numerically larger: reversed linkage)
            0.0, // servo AT MAX ANGLE
            60.0, 30.0, 160.0, 1.0, 1000.0, 1000.0, 1450.0, 0.12, 0.004, 10.0);

    assertEquals(
        "servo at the min-angle end must map back to the min angle",
        15.0,
        BallisticsModel.servoPosToAngle(0.85, reversed),
        1e-6);
    assertEquals(
        "servo 0.0 is the steepest arc, so it must map to the max angle",
        65.0,
        BallisticsModel.servoPosToAngle(0.0, reversed),
        1e-6);
    assertEquals(
        "midpoint of travel maps to the mid angle",
        40.0,
        BallisticsModel.servoPosToAngle(0.425, reversed),
        1e-6);

    // Steeper must mean a NUMERICALLY SMALLER servo position on this linkage.
    assertTrue(
        "a steeper angle must command a smaller servo position when the linkage is reversed",
        BallisticsModel.angleToServoPos(60.0, reversed)
            < BallisticsModel.angleToServoPos(20.0, reversed));

    // Round-trip through both directions.
    for (double deg : new double[] {15.0, 25.0, 40.0, 55.0, 65.0}) {
      double servo = BallisticsModel.angleToServoPos(deg, reversed);
      assertEquals(
          "angle->servo->angle must round-trip on a reversed linkage",
          deg,
          BallisticsModel.servoPosToAngle(servo, reversed),
          1e-6);
    }

    assertEquals("ordered travel low", 0.0, reversed.servoTravelLow(), 1e-9);
    assertEquals("ordered travel high", 0.85, reversed.servoTravelHigh(), 1e-9);
  }

  @Test
  public void testTrajectoryIntegrationStability() {
    BallisticsModel.TrajectoryResult result =
        BallisticsModel.integrateTrajectory(35.0, 72.0, params);

    assertTrue("trajectory should reach target distance", result.reachedDistance());
    assertTrue("flight time should be positive", result.flightTimeSec() > 0.0);
    assertTrue("flight time should be realistic (< 2.0s)", result.flightTimeSec() < 2.0);
  }

  @Test
  public void testLoftedElevationSolver() {
    BallisticsModel.SolvedElevation solved = BallisticsModel.solveLoftedElevation(84.0, params);

    assertNotNull("solved elevation should not be null", solved);
    assertTrue(
        "elevation angle should be within hood limits",
        solved.elevationAngleDeg() >= params.minHoodAngleDeg()
            && solved.elevationAngleDeg() <= params.maxHoodAngleDeg());

    assertTrue(
        "hood servo position should be within servo bounds",
        solved.hoodServoPosition() >= params.servoTravelLow()
            && solved.hoodServoPosition() <= params.servoTravelHigh());

    assertTrue("flight time should be positive", solved.flightTimeSec() > 0.0);
  }

  @Test
  public void testContinuousServoPositionMapping() {
    double minPos = BallisticsModel.angleToServoPos(params.minHoodAngleDeg(), params);
    double maxPos = BallisticsModel.angleToServoPos(params.maxHoodAngleDeg(), params);

    assertEquals(params.minHoodServoPos(), minPos, 1e-6);
    assertEquals(params.maxHoodServoPos(), maxPos, 1e-6);
  }

  @Test
  public void testSolveShotStaysWithinFlywheelRpmBounds() {
    for (double dist : new double[] {36.0, 60.0, 84.0, 120.0, 150.0}) {
      BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(dist, params);

      double rpm = params.v0ToRpm(shot.exitVelocityIps());
      assertTrue(
          "solved RPM at " + dist + " in must not fall below preferred_shot_rpm, was " + rpm,
          rpm >= params.preferredShotRpm() - 1e-6);
      assertTrue(
          "solved RPM at " + dist + " in must not exceed max_shot_rpm, was " + rpm,
          rpm <= params.maxShotRpm() + 1e-6);
      assertTrue(
          "hood servo position must stay within bounds at " + dist + " in",
          shot.hoodServoPosition() >= params.servoTravelLow()
              && shot.hoodServoPosition() <= params.servoTravelHigh());
    }
  }

  /**
   * Robot-representative parameters: v0 250 in/s at 1000 RPM, matching config.yaml. Weak enough
   * that the hood alone cannot cover the whole field, so both halves of the policy are exercised.
   */
  private static BallisticsParameters realisticParams() {
    return new BallisticsParameters(
        250.0, 0.0012, 0.015, 15.0, 45.0, 386.088, 15.0, 65.0, 0.0, 0.85, 60.0, 30.0, 150.0, 1.0,
        1000.0, 1000.0, 1450.0, 0.12, 0.004, 10.0);
  }

  @Test
  public void testHoodAloneCoversCloseRangeAtPreferredRpm() {
    // Regression guard: an earlier minimum-energy policy dropped a 48in shot to ~845 RPM, which
    // tested far too weak on the robot. Close and mid range must stay parked at the preferred
    // speed and let the hood do the aiming.
    BallisticsParameters p = realisticParams();

    for (double dist : new double[] {30.0, 48.0, 60.0, 72.0}) {
      double rpm = p.v0ToRpm(BallisticsModel.solveShot(dist, p).exitVelocityIps());
      assertEquals(
          "hood should cover " + dist + " in at the preferred RPM, not slow the flywheel down",
          p.preferredShotRpm(),
          rpm,
          1e-6);
    }
  }

  @Test
  public void testFlywheelRisesOnlyBeyondHoodAuthority() {
    BallisticsParameters p = realisticParams();

    double previous = 0.0;
    for (double dist : new double[] {30.0, 48.0, 72.0, 96.0, 120.0, 144.0}) {
      double rpm = p.v0ToRpm(BallisticsModel.solveShot(dist, p).exitVelocityIps());
      assertTrue(
          "solved RPM must never drop below the preferred speed (was " + rpm + " at " + dist + ")",
          rpm >= p.preferredShotRpm() - 1e-6);
      assertTrue("solved RPM must not decrease with distance at " + dist, rpm >= previous - 1e-6);
      previous = rpm;
    }

    double farRpm = p.v0ToRpm(BallisticsModel.solveShot(144.0, p).exitVelocityIps());
    assertTrue(
        "a 144in target is past the hood's authority at 1000 RPM and must raise the flywheel",
        farRpm > p.preferredShotRpm() + 1.0);
  }

  @Test
  public void testMarginKeepsLongShotsOffTheDegenerateLob() {
    // Right at the edge of a speed's range the only solution left is a near-vertical lob. Without
    // the margin, a 96in shot solved to a 63deg hood and 0.94s of hang time.
    BallisticsParameters p = realisticParams();

    BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(96.0, p);

    assertTrue(
        "96in solution should not degenerate into a lob, got " + shot.elevationAngleDeg() + " deg",
        shot.elevationAngleDeg() < 50.0);
    assertTrue(
        "96in flight time should stay well under a second, got " + shot.flightTimeSec(),
        shot.flightTimeSec() < 0.7);
  }

  @Test
  public void testSolveShotReachesGoalHeightAtSolvedSpeed() {
    for (double dist : new double[] {48.0, 96.0, 144.0}) {
      BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(dist, params);
      assertTrue("shot at " + dist + " in should be feasible", shot.feasible());

      BallisticsModel.TrajectoryResult flown =
          BallisticsModel.integrateTrajectory(
              shot.elevationAngleDeg(), dist, params.withV0(shot.exitVelocityIps()));

      assertTrue("solved trajectory must reach the target distance", flown.reachedDistance());
      assertEquals(
          "solved trajectory must arrive at goal height at " + dist + " in",
          params.goalHeightInches(),
          flown.heightAtTargetInches(),
          1.0);
    }
  }

  @Test
  public void testSolveShotReportsInfeasibleBeyondFlywheelRange() {
    // Cap the flywheel far below what any real shot needs, so nothing is reachable.
    BallisticsParameters crippled =
        new BallisticsParameters(
            360.0, 0.0012, 0.015, 15.0, 45.0, 386.088, 15.0, 65.0, 0.15, 0.85, 60.0, 30.0, 160.0,
            1.0, 1000.0, 50.0, 60.0, 0.12, 0.004, 10.0);

    BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(140.0, crippled);

    assertFalse("unreachable target must report infeasible", shot.feasible());
    assertTrue(
        "infeasible reason should name the RPM ceiling", shot.infeasibleReason().contains("RPM"));
  }

  @Test
  public void testLevenbergMarquardtParameterFitting() {
    java.util.List<double[]> points = new java.util.ArrayList<>();
    points.add(new double[] {48.0, 52.0});
    points.add(new double[] {72.0, 44.0});
    points.add(new double[] {96.0, 36.0});

    BallisticsModel.FittedBallistics fitted = BallisticsModel.fitParameters(points, params);

    assertNotNull("fitted parameters should not be null", fitted);
    assertTrue("fitted v0 should be positive", fitted.v0() > 0.0);
    assertTrue("fitted k drag should be positive", fitted.k() > 0.0);
    assertTrue("fitted magnus L lift should be positive", fitted.magnusL() > 0.0);
  }

  @Test
  public void testFitParametersWithVariableRpm() {
    BallisticsParameters trueParams =
        new BallisticsParameters(
            420.0, 0.0012, 0.045, 10.0, 30.0, 386.088, 10.0, 55.0, 0.0, 1.0, 60.0, 30.0, 160.0, 1.0,
            1400.0, 1400.0, 2200.0, 0.15, 0.004, 10.0);

    double refRpm = trueParams.v0ReferenceRpm();
    double[] distances = new double[] {48.0, 72.0, 96.0, 120.0, 144.0, 150.0};
    double[] rpms = new double[] {1400.0, 1400.0, 1400.0, 1400.0, 1600.0, 1700.0};

    java.util.List<double[]> variableRpmPoints = new java.util.ArrayList<>();
    for (int i = 0; i < distances.length; i++) {
      double dist = distances[i];
      double rpm = rpms[i];
      double exitV0 = trueParams.v0() * (rpm / refRpm);
      BallisticsModel.SolvedElevation solved =
          BallisticsModel.solveLoftedElevation(dist, trueParams.withV0(exitV0));
      variableRpmPoints.add(new double[] {dist, solved.elevationAngleDeg(), rpm});
    }

    BallisticsParameters initialGuess = trueParams.withV0(380.0).withDragAndLift(0.0020, 0.020);

    BallisticsModel.FittedBallistics fitted =
        BallisticsModel.fitParametersWithRpm(variableRpmPoints, initialGuess);

    assertNotNull("fitted ballistics should not be null", fitted);
    assertTrue(
        "RMS residual error should be under 0.1 inches, was " + fitted.residualRmsInches(),
        fitted.residualRmsInches() < 0.1);
    assertEquals("fitted v0 should recover true v0 within 1%", 420.0, fitted.v0(), 4.2);
  }
}
