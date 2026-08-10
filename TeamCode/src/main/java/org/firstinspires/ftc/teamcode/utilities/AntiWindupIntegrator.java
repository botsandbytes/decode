package org.firstinspires.ftc.teamcode.utilities;

public final class AntiWindupIntegrator {

  /** Largest plausible loop period (seconds); longer gaps are treated as a scheduling stall. */
  private static final double MAX_DT_SECONDS = 0.25;

  private double accumulator = 0.0;

  /** Clears the accumulated integral. Call when the mechanism stops or the setpoint jumps. */
  public void reset() {
    accumulator = 0.0;
  }

  /** Current accumulated error-seconds, before multiplication by {@code ki}. */
  public double getAccumulator() {
    return accumulator;
  }

  /**
   * Advances the integrator and returns its contribution to the motor command.
   *
   * @param error setpoint minus measurement, in ticks/s
   * @param dtSeconds loop period; non-positive or implausibly long values are skipped
   * @param ki integral gain; zero or negative disables the term entirely
   * @param band only integrate when {@code |error| <= band}; non-positive means always integrate
   * @param maxContribution bound on the absolute returned contribution, in power units
   * @param saturationSign +1 if the command is railed high, -1 if railed low, 0 if in range
   * @return {@code ki * accumulator}, clamped to {@code +/-maxContribution}
   */
  public double update(
      double error,
      double dtSeconds,
      double ki,
      double band,
      double maxContribution,
      int saturationSign) {

    if (ki <= 0.0) {
      accumulator = 0.0;
      return 0.0;
    }

    boolean dtUsable = dtSeconds > 0.0 && dtSeconds <= MAX_DT_SECONDS;
    boolean withinBand = band <= 0.0 || Math.abs(error) <= band;
    // Charging further into a rail cannot move the output, so freeze that direction.
    boolean pushingIntoRail =
        (saturationSign > 0 && error > 0.0) || (saturationSign < 0 && error < 0.0);

    if (dtUsable && withinBand && !pushingIntoRail) {
      accumulator += error * dtSeconds;
    }

    double limit = maxContribution > 0.0 ? maxContribution / ki : Double.MAX_VALUE;
    accumulator = Math.max(-limit, Math.min(limit, accumulator));

    return ki * accumulator;
  }
}
