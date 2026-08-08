package org.firstinspires.ftc.teamcode.ballistics;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;

/**
 * Pure, stateless shot solver. Resolves the moving-shot lead geometry, then reads the hood position
 * and flywheel setpoint straight out of the measured {@link ShotTable}.
 */
public class ShotSolver {

  /**
   * Computes the complete ShotSolution for a given loop's robot state.
   *
   * @param inputs Live robot pose, velocity, and goal position
   * @param table Measured distance-to-shot lookup
   * @param params Gates and lead calibration (speed limit, valid range, lead bias, flight time)
   * @param lastFlightTime Previous loop's flight time, warm-starting the lead fixed point
   */
  public static ShotSolution solve(
      ShotInputs inputs, ShotTable table, BallisticsParameters params, double lastFlightTime) {
    if (inputs == null || inputs.robotPose() == null || table == null || params == null) {
      return new ShotSolution(
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, "Null inputs, table, or parameters");
    }

    Pose pose = inputs.robotPose();
    Pose vel = inputs.robotVelocity() != null ? inputs.robotVelocity() : new Pose(0, 0, 0);

    double rx = pose.getX();
    double ry = pose.getY();
    double gx = inputs.targetGoalX();
    double gy = inputs.targetGoalY();

    double vx = vel.getX();
    double vy = vel.getY();
    double robotSpeed = Math.hypot(vx, vy);

    if (robotSpeed > params.maxMovingSpeedIps()) {
      return new ShotSolution(
          table.lookup(Math.hypot(gy - ry, gx - rx)).hoodServoPosition(),
          0.0,
          0.0,
          Math.atan2(gy - ry, gx - rx),
          0.0,
          0.0,
          Math.hypot(gy - ry, gx - rx),
          false,
          String.format(
              "Robot speed %.1f in/s exceeds max speed gate %.1f in/s",
              robotSpeed, params.maxMovingSpeedIps()));
    }

    // Lead: aim at where the goal will be relative to the robot after the ring's flight. Flight
    // time comes from a flat seconds-per-inch constant rather than an integrated trajectory —
    // the table gives no flight time, and lead only shifts the azimuth by a small angle, so a
    // crude estimate is adequate where a wrong hood position would not be.
    //
    // Below leadMinSpeedIps the lead is dropped entirely. A robot that is holding position still
    // reports a small, noisy velocity, and that noise runs straight into the aim azimuth: the
    // offset is velocity * flightTime, so ~10 in/s of estimator noise swings the commanded bearing
    // by more than the turret's 1-2 deg settle tolerance. The turret then never reaches its target
    // and hunts continuously. Leading a robot that is barely moving buys nothing anyway.
    double flightTime = (lastFlightTime > 0.05 && lastFlightTime < 2.0) ? lastFlightTime : 0.4;
    boolean applyLead = robotSpeed >= params.leadMinSpeedIps();
    double virtX = gx;
    double virtY = gy;
    double virtDist = Math.hypot(gy - ry, gx - rx);

    for (int iter = 0; iter < 3; iter++) {
      virtX = applyLead ? gx - (vx * flightTime * params.leadBiasGain()) : gx;
      virtY = applyLead ? gy - (vy * flightTime * params.leadBiasGain()) : gy;
      virtDist = Math.hypot(virtY - ry, virtX - rx);

      double next = virtDist * params.flightTimeSecPerInch();
      boolean converged = Math.abs(next - flightTime) < 1e-3;
      flightTime = next;
      if (converged || !applyLead) {
        break;
      }
    }

    ShotTable.Shot shot = table.lookup(virtDist);

    boolean distanceValid =
        virtDist >= params.minValidDistanceInches() && virtDist <= params.maxValidDistanceInches();

    String reason = "Valid";
    if (!distanceValid) {
      reason =
          String.format(
              "Virtual distance %.1f in outside calibrated range [%.1f, %.1f]",
              virtDist, params.minValidDistanceInches(), params.maxValidDistanceInches());
    } else if (!shot.withinCalibratedRange()) {
      reason = shot.reason();
    }

    double azimuthRad = Math.atan2(virtY - ry, virtX - rx);

    return new ShotSolution(
        shot.hoodServoPosition(),
        shot.rpm(),
        params.rpmToV0(shot.rpm()),
        azimuthRad,
        flightTime,
        0.0,
        virtDist,
        distanceValid && shot.withinCalibratedRange(),
        reason);
  }
}
