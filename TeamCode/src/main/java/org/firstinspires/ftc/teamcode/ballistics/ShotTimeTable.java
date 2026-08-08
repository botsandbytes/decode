package org.firstinspires.ftc.teamcode.ballistics;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Measured distance-to-shoot-window lookup: how long a full magazine actually takes to leave.
 *
 * <p>Every row is a shot the Shot Timing Tuner counted ball by ball, from the moment the shot was
 * armed to 250 ms after the last ball cleared the flywheel. Nothing is modelled — the shot's
 * duration depends on flywheel spin-up, per-ball sag, and how fast the feed gate re-arms, none of
 * which is predictable from first principles on this robot.
 *
 * <p>The curve is not monotonic in the direction anyone expects: a close shot takes <i>longer</i>
 * than a far one (4.8 s at 68 in against 2.9 s at 122 in). A faster flywheel stores more energy, so
 * each ball costs a smaller fraction of it and the gate re-arms sooner. That alone is the reason a
 * single flat {@code shoot_wait_ms} cannot be right at both ends of the field.
 *
 * <p>Interpolation is linear between rows and <b>clamped</b> outside them. Clamping is the safe
 * direction here, unlike in {@link ShotTable} where an uncalibrated distance is refused: a shoot
 * window that is too long only costs autonomous time, while one that is too short silently leaves
 * balls in the robot.
 */
public final class ShotTimeTable {

  private final double[] distances;
  private final int[] windowMs;

  private ShotTimeTable(double[] distances, int[] windowMs) {
    this.distances = distances;
    this.windowMs = windowMs;
  }

  /**
   * Builds the table from the flat {@code distance, window_ms} pairs in config, or returns null
   * when none are configured so the caller can fall back to the flat {@code shoot_wait_ms}.
   *
   * @throws IllegalStateException if the rows are malformed, since a mis-parsed table would bound
   *     every autonomous scoring cycle with an arbitrary number.
   */
  public static ShotTimeTable fromConfig() {
    return fromPoints(config.auto.shot_time_points);
  }

  /** Builds a table from flat {@code distance, window_ms} pairs; null when empty. */
  public static ShotTimeTable fromPoints(double[] points) {
    if (points == null || points.length == 0) {
      return null;
    }
    if (points.length % 2 != 0) {
      throw new IllegalStateException(
          "auto.shot_time_points must be flat distance,window_ms pairs but had "
              + points.length
              + " values");
    }

    int n = points.length / 2;
    double[] d = new double[n];
    int[] w = new int[n];
    for (int i = 0; i < n; i++) {
      d[i] = points[i * 2];
      w[i] = (int) Math.round(points[i * 2 + 1]);
      if (i > 0 && d[i] <= d[i - 1]) {
        throw new IllegalStateException(
            String.format(
                "shot time table distances must strictly increase, but row %d (%.1f in) does not"
                    + " follow row %d (%.1f in)",
                i, d[i], i - 1, d[i - 1]));
      }
      if (w[i] < 0) {
        throw new IllegalStateException(
            String.format("shot time table window at %.1f in is negative (%d ms)", d[i], w[i]));
      }
    }
    return new ShotTimeTable(d, w);
  }

  public int size() {
    return distances.length;
  }

  public double minDistance() {
    return distances[0];
  }

  public double maxDistance() {
    return distances[distances.length - 1];
  }

  /** Shoot window (ms) for a distance, interpolated between rows and clamped outside them. */
  public int lookupMs(double distanceInches) {
    if (!Double.isFinite(distanceInches) || distanceInches <= distances[0]) {
      return windowMs[0];
    }
    int last = distances.length - 1;
    if (distanceInches >= distances[last]) {
      return windowMs[last];
    }

    int hi = 1;
    while (hi < last && distances[hi] < distanceInches) {
      hi++;
    }
    int lo = hi - 1;
    double t = (distanceInches - distances[lo]) / (distances[hi] - distances[lo]);
    return (int) Math.round(windowMs[lo] + t * (windowMs[hi] - windowMs[lo]));
  }

  /**
   * Window for a distance, falling back to {@code auto.shoot_wait_ms} when no table is configured.
   *
   * <p>The one entry point autonomous should use — it never throws and never returns zero, because
   * a shot bounded by nothing is a scoring cycle that never ends.
   */
  public static int windowMsFor(double distanceInches) {
    try {
      ShotTimeTable table = fromConfig();
      return table != null ? table.lookupMs(distanceInches) : config.auto.shoot_wait_ms;
    } catch (RuntimeException e) {
      return config.auto.shoot_wait_ms;
    }
  }
}
