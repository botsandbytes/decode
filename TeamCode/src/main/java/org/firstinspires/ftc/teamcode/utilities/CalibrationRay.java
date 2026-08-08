package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;

/**
 * The ray that calibration OpModes walk out along from a goal, and the legal drive box bounding it.
 *
 * <p>Shared by every calibration that needs to stand at a known distance from the goal, so they all
 * shoot along the same bearing and their results are comparable. Distances measured on one ray and
 * applied on another are only interchangeable because the shot depends on distance alone — keeping
 * one ray is what makes that assumption safe to keep making.
 *
 * <p>The ray is aimed at the corner {@code (MAX_TARGET_X, MIN_TARGET_Y)} on purpose: distance from
 * the goal grows with both +x and -y, so that corner is the farthest reachable point in the box and
 * this bearing maximises calibration range.
 */
public final class CalibrationRay {

  private CalibrationRay() {}

  /** Legal drive box for a calibration target, in Pedro Pathing field coordinates. */
  public static final double MAX_TARGET_X = 85.0;

  public static final double MIN_TARGET_Y = 11.0;

  private static final double RAY_END_X = MAX_TARGET_X;
  private static final double RAY_END_Y = MIN_TARGET_Y;

  /** Unit vector pointing from the goal out along the calibration ray, as {@code {x, y}}. */
  public static double[] unitFromGoal(double goalX, double goalY) {
    double dx = RAY_END_X - goalX;
    double dy = RAY_END_Y - goalY;
    double length = Math.max(1e-6, Math.hypot(dx, dy));
    return new double[] {dx / length, dy / length};
  }

  /**
   * Distance along the ray at which it first leaves the legal drive box. Each bound contributes a
   * limit only when the ray actually travels toward it.
   *
   * <p>A requested distance past this is not calibratable along this bearing and must be skipped,
   * not clamped. Clamping x and y independently slides the target off the ray entirely: a requested
   * 144 in became a real 124.8 in on a different bearing, and got recorded as a 144 in trial.
   */
  public static double maxDistance(double goalX, double goalY) {
    double[] unit = unitFromGoal(goalX, goalY);
    double limit = Double.MAX_VALUE;
    if (unit[0] > 1e-6) {
      limit = Math.min(limit, (MAX_TARGET_X - goalX) / unit[0]);
    }
    if (unit[1] < -1e-6) {
      limit = Math.min(limit, (MIN_TARGET_Y - goalY) / unit[1]);
    }
    return limit;
  }

  /** Pose {@code distanceInches} out along the ray, headed back at the goal. */
  public static Pose waypoint(double goalX, double goalY, double distanceInches) {
    double[] unit = unitFromGoal(goalX, goalY);
    double x = goalX + distanceInches * unit[0];
    double y = goalY + distanceInches * unit[1];
    return new Pose(x, y, Math.atan2(goalY - y, goalX - x));
  }
}
