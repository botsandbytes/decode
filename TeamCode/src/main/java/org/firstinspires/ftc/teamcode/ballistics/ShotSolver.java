package org.firstinspires.ftc.teamcode.ballistics;

import com.pedropathing.geometry.Pose;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;

public class ShotSolver {

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
              Locale.ROOT,
              "Robot speed %.1f in/s exceeds max speed gate %.1f in/s",
              robotSpeed,
              params.maxMovingSpeedIps()));
    }

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
              Locale.ROOT,
              "Virtual distance %.1f in outside calibrated range [%.1f, %.1f]",
              virtDist,
              params.minValidDistanceInches(),
              params.maxValidDistanceInches());
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
