package org.firstinspires.ftc.teamcode.ballistics;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;

public final class ShotTimeTable {

  private final double[] rpms;
  private final int[] windowMs;

  private ShotTimeTable(double[] rpms, int[] windowMs) {
    this.rpms = rpms;
    this.windowMs = windowMs;
  }

  /**
   * Builds the table from the flat {@code target_rpm, window_ms} pairs in config, or returns null
   * when none are configured so the caller can fall back to the flat {@code shoot_wait_ms}.
   *
   * @throws IllegalStateException if the rows are malformed, since a mis-parsed table would bound
   *     every autonomous scoring cycle with an arbitrary number.
   */
  public static ShotTimeTable fromConfig() {
    return fromPoints(config.auto.shot_time_points);
  }

  /** Builds a table from flat {@code target_rpm, window_ms} pairs; null when empty. */
  public static ShotTimeTable fromPoints(double[] points) {
    if (points == null || points.length == 0) {
      return null;
    }
    if (points.length % 2 != 0) {
      throw new IllegalStateException(
          "auto.shot_time_points must be flat rpm,window_ms pairs but had "
              + points.length
              + " values");
    }

    int n = points.length / 2;
    double[] r = new double[n];
    int[] w = new int[n];
    for (int i = 0; i < n; i++) {
      r[i] = points[i * 2];
      w[i] = (int) Math.round(points[i * 2 + 1]);
      if (i > 0 && r[i] <= r[i - 1]) {
        throw new IllegalStateException(
            String.format(
                "shot time table rpms must strictly increase, but row %d (%.1f rpm) does not"
                    + " follow row %d (%.1f rpm)",
                i, r[i], i - 1, r[i - 1]));
      }
      if (w[i] < 0) {
        throw new IllegalStateException(
            String.format("shot time table window at %.1f rpm is negative (%d ms)", r[i], w[i]));
      }
    }
    return new ShotTimeTable(r, w);
  }

  public int size() {
    return rpms.length;
  }

  public double minRpm() {
    return rpms[0];
  }

  public double maxRpm() {
    return rpms[rpms.length - 1];
  }

  /** Shoot window (ms) for a target RPM, interpolated between rows and clamped outside them. */
  public int lookupMs(double targetRpm) {
    if (!Double.isFinite(targetRpm) || targetRpm <= rpms[0]) {
      return windowMs[0];
    }
    int last = rpms.length - 1;
    if (targetRpm >= rpms[last]) {
      return windowMs[last];
    }

    int hi = 1;
    while (hi < last && rpms[hi] < targetRpm) {
      hi++;
    }
    int lo = hi - 1;
    double t = (targetRpm - rpms[lo]) / (rpms[hi] - rpms[lo]);
    return (int) Math.round(windowMs[lo] + t * (windowMs[hi] - windowMs[lo]));
  }

  /**
   * Window for a target RPM, falling back to {@code auto.shoot_wait_ms} when no table is
   * configured.
   *
   * <p>The one entry point autonomous should use — it never throws and never returns zero, because
   * a shot bounded by nothing is a scoring cycle that never ends.
   */
  public static int windowMsFor(double targetRpm) {
    try {
      ShotTimeTable table = fromConfig();
      return table != null ? table.lookupMs(targetRpm) : config.auto.shoot_wait_ms;
    } catch (RuntimeException e) {
      return config.auto.shoot_wait_ms;
    }
  }
}
