package org.firstinspires.ftc.teamcode.ballistics;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;

public final class ShotTable {

  /** One interpolated shot: what to command, and whether the distance was actually calibrated. */
  public record Shot(
      double hoodServoPosition, double rpm, boolean withinCalibratedRange, String reason) {}

  private final double[] distances;
  private final double[] hoodServo;
  private final double[] rpm;

  private ShotTable(double[] distances, double[] hoodServo, double[] rpm) {
    this.distances = distances;
    this.hoodServo = hoodServo;
    this.rpm = rpm;
  }

  /**
   * Builds the table from the flat {@code distance, hood_servo, rpm} triples in config.
   *
   * @throws IllegalStateException if the row data is malformed, since a silently mis-parsed shot
   *     table would command arbitrary hood and flywheel values on every shot of a match.
   */
  public static ShotTable fromConfig() {
    return fromPoints(config.shooter.shot_table.points);
  }

  /** Builds a table from flat {@code distance, hood_servo, rpm} triples. */
  public static ShotTable fromPoints(double[] points) {
    if (points == null || points.length == 0) {
      throw new IllegalStateException(
          "shooter.shot_table.points is empty — run the Ballistics Calibration OpMode");
    }
    if (points.length % 3 != 0) {
      throw new IllegalStateException(
          "shooter.shot_table.points must be flat distance,hood_servo,rpm triples but had "
              + points.length
              + " values");
    }

    int n = points.length / 3;
    double[] d = new double[n];
    double[] h = new double[n];
    double[] r = new double[n];
    for (int i = 0; i < n; i++) {
      d[i] = points[i * 3];
      h[i] = points[i * 3 + 1];
      r[i] = points[i * 3 + 2];
      if (i > 0 && d[i] <= d[i - 1]) {
        throw new IllegalStateException(
            String.format(
                "shot table distances must strictly increase, but row %d (%.1f in) does not follow"
                    + " row %d (%.1f in)",
                i, d[i], i - 1, d[i - 1]));
      }
    }
    return new ShotTable(d, h, r);
  }

  /** Lowest calibrated distance (inches). */
  public double minDistance() {
    return distances[0];
  }

  /** Highest calibrated distance (inches). */
  public double maxDistance() {
    return distances[distances.length - 1];
  }

  public int size() {
    return distances.length;
  }

  /**
   * Looks up the hood position and flywheel setpoint for a distance, interpolating between the two
   * bracketing rows. Outside the calibrated range the nearest end row is returned with {@code
   * withinCalibratedRange == false} so callers can gate on it rather than acting on a guess.
   */
  public Shot lookup(double distanceInches) {
    if (distanceInches <= distances[0]) {
      return new Shot(
          hoodServo[0],
          rpm[0],
          distanceInches >= distances[0] - 1e-9,
          String.format(
              "%.1f in is below the calibrated range (min %.1f in)", distanceInches, distances[0]));
    }
    int last = distances.length - 1;
    if (distanceInches >= distances[last]) {
      return new Shot(
          hoodServo[last],
          rpm[last],
          distanceInches <= distances[last] + 1e-9,
          String.format(
              "%.1f in is beyond the calibrated range (max %.1f in)",
              distanceInches, distances[last]));
    }

    int hi = 1;
    while (hi < last && distances[hi] < distanceInches) {
      hi++;
    }
    int lo = hi - 1;
    double t = (distanceInches - distances[lo]) / (distances[hi] - distances[lo]);
    return new Shot(
        hoodServo[lo] + t * (hoodServo[hi] - hoodServo[lo]),
        rpm[lo] + t * (rpm[hi] - rpm[lo]),
        true,
        "Valid");
  }
}
