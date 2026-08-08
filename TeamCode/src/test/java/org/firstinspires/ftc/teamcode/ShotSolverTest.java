package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.ballistics.ShotSolver;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.junit.Before;
import org.junit.Test;

public class ShotSolverTest {

  private BallisticsParameters params;
  private ShotTable table;

  @Before
  public void setUp() {
    params =
        new BallisticsParameters(
            360.0, // v0 = 360 in/s
            0.0012, // k drag
            0.015, // L Magnus lift
            15.0, // launch height
            45.0, // goal height
            386.088, // g
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
    table =
        ShotTable.fromPoints(
            new double[] {
              48.0, 0.100, 900.0,
              72.0, 0.200, 1000.0,
              96.0, 0.300, 1100.0,
              140.0, 0.500, 1300.0
            });
  }

  @Test
  public void testStaticRobotSolution() {
    ShotInputs inputs = new ShotInputs(new Pose(0, 0, 0), new Pose(0, 0, 0), 72.0, 0.0);

    ShotSolution solution = ShotSolver.solve(inputs, table, params, 0.4);

    assertTrue("static shot solution should be valid", solution.isValid());
    assertEquals(
        "static azimuth to goal (72, 0) from (0, 0) should be 0",
        0.0,
        solution.targetAzimuthRad(),
        1e-3);
    assertTrue("flight time should be positive", solution.predictedFlightTimeSec() > 0.0);
  }

  @Test
  public void testSolutionComesStraightFromTheMeasuredTable() {
    ShotInputs inputs = new ShotInputs(new Pose(0, 0, 0), new Pose(0, 0, 0), 72.0, 0.0);

    ShotSolution solution = ShotSolver.solve(inputs, table, params, 0.4);

    assertEquals(
        "hood is the table row, not a solved angle", 0.200, solution.targetHoodPosition(), 1e-9);
    assertEquals("rpm is the table row", 1000.0, solution.targetRpm(), 1e-9);
  }

  @Test
  public void testInterpolatesBetweenCalibratedDistances() {
    ShotInputs inputs = new ShotInputs(new Pose(0, 0, 0), new Pose(0, 0, 0), 84.0, 0.0);

    ShotSolution solution = ShotSolver.solve(inputs, table, params, 0.4);

    assertTrue(solution.isValid());
    assertEquals("halfway between the 72 and 96 rows", 0.250, solution.targetHoodPosition(), 1e-9);
    assertEquals(1050.0, solution.targetRpm(), 1e-9);
  }

  @Test
  public void testMovingRobotLeadCorrection() {
    // Robot moving sideways in Y at +30 in/s
    ShotInputs inputs = new ShotInputs(new Pose(0, 0, 0), new Pose(0, 30.0, 0), 72.0, 0.0);

    ShotSolution solution = ShotSolver.solve(inputs, table, params, 0.4);

    assertTrue("moving shot solution below max speed gate should be valid", solution.isValid());
    assertTrue(
        "target azimuth should lead into negative Y direction to counteract +Y velocity",
        solution.targetAzimuthRad() < 0.0);
  }

  @Test
  public void testFartherShotsGetMoreFlywheelAndFlatterHoodFromTheTable() {
    ShotInputs near = new ShotInputs(new Pose(0, 0, 0), new Pose(0, 0, 0), 48.0, 0.0);
    ShotInputs far = new ShotInputs(new Pose(0, 0, 0), new Pose(0, 0, 0), 140.0, 0.0);

    ShotSolution nearSol = ShotSolver.solve(near, table, params, 0.4);
    ShotSolution farSol = ShotSolver.solve(far, table, params, 0.4);

    assertTrue("farther shot needs more flywheel", farSol.targetRpm() > nearSol.targetRpm());
    assertTrue(
        "farther shot needs a flatter arc, which on this linkage is a larger servo position",
        farSol.targetHoodPosition() > nearSol.targetHoodPosition());
  }

  /** Past the last measured row there is nothing to interpolate, so the solution must not pass. */
  @Test
  public void testBeyondCalibratedRangeIsInvalid() {
    ShotInputs inputs = new ShotInputs(new Pose(0, 0, 0), new Pose(0, 0, 0), 155.0, 0.0);

    ShotSolution solution = ShotSolver.solve(inputs, table, params, 0.4);

    assertFalse("a distance past the table must not report as valid", solution.isValid());
    assertEquals(
        "hood clamps to the last measured row rather than extrapolating",
        0.500,
        solution.targetHoodPosition(),
        1e-9);
  }

  /**
   * A robot holding position still reports a small, noisy velocity. Leading on that noise swung the
   * commanded aim azimuth by more than the turret's 1-2 degree settle tolerance, so the turret
   * never reached its target and hunted continuously — which is what made the turret look like it
   * was moving randomly during ballistics calibration while behaving fine under a plain HOLD.
   */
  @Test
  public void testLowSpeedVelocityNoiseDoesNotMoveTheAimAzimuth() {
    Pose pose = new Pose(0, 0, 0);
    double still =
        ShotSolver.solve(new ShotInputs(pose, new Pose(0, 0, 0), 72.0, 0.0), table, params, 0.4)
            .targetAzimuthRad();

    // Sweep plausible estimator noise below the lead threshold; aim must not budge.
    for (double noise : new double[] {-9.0, -5.0, -2.0, 2.0, 5.0, 9.0}) {
      double jittered =
          ShotSolver.solve(
                  new ShotInputs(pose, new Pose(0, noise, 0), 72.0, 0.0), table, params, 0.4)
              .targetAzimuthRad();
      assertEquals(
          "velocity noise of " + noise + " in/s must not steer the turret", still, jittered, 1e-9);
    }
  }

  /** Above the threshold the robot is genuinely moving and lead must still be applied. */
  @Test
  public void testLeadStillAppliesOnceActuallyMoving() {
    Pose pose = new Pose(0, 0, 0);
    double still =
        ShotSolver.solve(new ShotInputs(pose, new Pose(0, 0, 0), 72.0, 0.0), table, params, 0.4)
            .targetAzimuthRad();
    double moving =
        ShotSolver.solve(new ShotInputs(pose, new Pose(0, 40.0, 0), 72.0, 0.0), table, params, 0.4)
            .targetAzimuthRad();

    assertTrue("a genuinely moving robot must still lead its shot", moving < still - 1e-6);
  }

  @Test
  public void testMaxSpeedGateEnforcement() {
    // Robot moving at 80 in/s (exceeds max speed gate 60 in/s)
    ShotInputs inputs = new ShotInputs(new Pose(0, 0, 0), new Pose(80.0, 0.0, 0), 72.0, 0.0);

    ShotSolution solution = ShotSolver.solve(inputs, table, params, 0.4);

    assertFalse("solution exceeding max speed gate must be invalid", solution.isValid());
    assertTrue(
        "validity reason should mention speed gate", solution.validityReason().contains("speed"));
  }
}
