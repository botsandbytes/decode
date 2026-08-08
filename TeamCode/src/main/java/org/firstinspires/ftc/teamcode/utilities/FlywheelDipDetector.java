package org.firstinspires.ftc.teamcode.utilities;

/**
 * Counts balls leaving the flywheel by the speed each one steals.
 *
 * <p>A ball entering the flywheel takes energy out of it, and the wheel slows for a few tens of
 * milliseconds until the controller puts it back. That dip is a direct mechanical consequence of
 * the event being counted, it arrives in the bulk read the loop already performs, and its size
 * scales with the setpoint — so one threshold works at every distance.
 *
 * <p>Dips are measured against a running reference of recent velocity, <b>not</b> against the
 * commanded target. During a feed the wheel does not recover to its setpoint between balls, so a
 * target-relative threshold sees one enormous dip for the whole burst and reports a single ball.
 *
 * <p>A dip <b>ends when the wheel rebounds off its own trough</b>, not when it climbs back near the
 * reference it started from. This is the difference between counting three balls and counting one.
 * Measured on this robot, a single ball drops the flywheel from 1020 to 740 ticks/s and the
 * controller needs about 1.7 s to bring it back within 2% of where it started — but the next ball
 * arrives at 1.4 s. Requiring a return to the old reference leaves the detector latched inside the
 * first dip for the whole magazine, and every later ball lands invisibly inside it. Rebounding a
 * few percent off the trough happens as soon as the wheel turns the corner, which is the moment the
 * ball is actually gone.
 *
 * <p>On leaving a dip the reference re-seeds to wherever the wheel recovered to, so the next ball
 * is judged against the speed it actually started from rather than against a setpoint the wheel
 * never gets back to.
 *
 * <p>What the trigger must clear, measured across 11 shots on two different flywheel controllers: a
 * real ball costs <b>12.2% to 26.4%</b> of flywheel speed, and the deepest thing that is not a ball
 * — the wheel coasting down from its post-shot overshoot, one encoder quantum per loop — reaches
 * <b>5.4%</b>. Nothing lands in between. A trigger set inside that gap separates them perfectly; a
 * trigger below it reports a phantom ball on every single shot, which is exactly what a 5% default
 * did.
 *
 * <p>Pure and hardware-free: the thresholds are the whole design and they need to be testable
 * without a robot and a hopper full of balls.
 */
public final class FlywheelDipDetector {

  private final double dipFraction;
  private final double reboundFraction;
  private final double baselineAlpha;
  private final long refractoryMs;

  private double baselineVelocity = Double.NaN;
  private double troughVelocity = Double.NaN;
  private boolean inDip = false;
  private long lastEventMs = Long.MIN_VALUE;
  private int eventCount = 0;
  private double lastDipFraction = 0.0;
  private double deepestDipFraction = 0.0;

  /**
   * @param dipFraction how far below the reference, as a fraction, the wheel must fall to count a
   *     ball
   * @param reboundFraction how far back up off the dip's own trough the wheel must climb before the
   *     next ball can register. This is what ends a dip; it is not measured against the reference.
   * @param baselineAlpha EMA weight per update for the reference speed while not in a dip
   * @param refractoryMs minimum spacing between counted events
   */
  public FlywheelDipDetector(
      double dipFraction, double reboundFraction, double baselineAlpha, long refractoryMs) {
    this.dipFraction = dipFraction;
    this.reboundFraction = reboundFraction;
    this.baselineAlpha = baselineAlpha;
    this.refractoryMs = refractoryMs;
  }

  /**
   * Feeds one velocity sample. Call once per loop while the flywheel is at speed.
   *
   * @param velocity measured flywheel velocity; sign is ignored, as everywhere else that reads it
   * @return true on the leading edge of a new dip, i.e. exactly once per ball
   */
  public boolean update(double velocity, long nowMs) {
    double speed = Math.abs(velocity);
    if (!Double.isFinite(speed)) {
      return false;
    }
    if (Double.isNaN(baselineVelocity) || baselineVelocity <= 0.0) {
      baselineVelocity = speed;
      return false;
    }

    lastDipFraction = (baselineVelocity - speed) / baselineVelocity;
    deepestDipFraction = Math.max(deepestDipFraction, lastDipFraction);

    boolean risingEdge = false;
    if (inDip) {
      troughVelocity = Math.min(troughVelocity, speed);
      if (troughVelocity > 0.0 && speed >= troughVelocity * (1.0 + reboundFraction)) {
        inDip = false;
        // Re-seed where the wheel came back to, not where it was before the ball.
        baselineVelocity = speed;
      }
    } else if (lastDipFraction >= dipFraction) {
      // Latched on entry regardless of the refractory window below: without this, an edge
      // suppressed as too-soon would fire again the moment the window expired, while the wheel was
      // still down inside the very same dip. The reference is deliberately left alone here —
      // averaging in the sample that opened the dip moves the very number the dip is measured
      // against, understating its depth.
      inDip = true;
      troughVelocity = speed;
      risingEdge = true;
    } else {
      // Asymmetric on purpose: follow the wheel up instantly, let it down slowly.
      //
      // A dip ends the moment the wheel turns the corner off its trough, which is still far below
      // where it started — so the reference has to climb with the recovery, or the next ball is
      // judged against a number from the bottom of the previous dip and vanishes. Following the
      // recovery upward keeps the reference at the recent peak, which is what a ball is a dip from.
      // Downward it decays slowly instead, so a wheel that settles into a sag is tracked without
      // that sag ever looking like an event.
      baselineVelocity =
          speed > baselineVelocity
              ? speed
              : baselineVelocity + baselineAlpha * (speed - baselineVelocity);
    }

    if (risingEdge && (lastEventMs == Long.MIN_VALUE || nowMs - lastEventMs >= refractoryMs)) {
      lastEventMs = nowMs;
      eventCount++;
      return true;
    }
    return false;
  }

  /** Reference speed the next dip is measured against; NaN until the first sample. */
  public double getBaselineVelocity() {
    return baselineVelocity;
  }

  /** Most recent sample's depth below baseline, as a fraction. Negative means above it. */
  public double getLastDipFraction() {
    return lastDipFraction;
  }

  /**
   * Deepest dip seen since the last reset.
   *
   * <p>This is how the threshold gets set without guessing: if real balls are bottoming out at 8%
   * and the threshold is 5%, it is in the right place; if the deepest dip all shot was 3%, the
   * threshold is above the signal and the count is going to read zero.
   */
  public double getDeepestDipFraction() {
    return deepestDipFraction;
  }

  public boolean isInDip() {
    return inDip;
  }

  public int getEventCount() {
    return eventCount;
  }

  /** Clears reference, count, and dip statistics for a new shot. */
  public void reset() {
    baselineVelocity = Double.NaN;
    troughVelocity = Double.NaN;
    inDip = false;
    lastEventMs = Long.MIN_VALUE;
    eventCount = 0;
    lastDipFraction = 0.0;
    deepestDipFraction = 0.0;
  }
}
