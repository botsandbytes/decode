package org.firstinspires.ftc.teamcode.ballistics;

import java.util.List;

/**
 * Least-squares identification of a flywheel's steady-state feedforward model {@code u = kV * v +
 * kS}, where {@code u} is motor power and {@code v} is velocity in ticks/s.
 *
 * <p>Pure math with no hardware dependencies so it can be unit tested on the JVM. Fitting across
 * the whole sweep — rather than forcing a line through a stiction point and a single full-power
 * point — keeps one bad sample from tilting the entire feedforward.
 */
public final class FlywheelFeedforwardFit {

  private FlywheelFeedforwardFit() {}

  /** A settled open-loop operating point. */
  public record Sample(double power, double velocityTicksPerSec) {}

  /**
   * Result of the fit.
   *
   * @param kV power per tick/s of velocity
   * @param kS power intercept (static friction)
   * @param rmsResidual root-mean-square residual, in power units
   * @param maxResidual worst absolute residual, in power units
   */
  public record Result(double kV, double kS, double rmsResidual, double maxResidual) {

    /** SDK-convention feedforward constant, scaled by the 32767 normalization factor. */
    public double f() {
      return kV * 32767.0;
    }

    /** Velocity at which the model predicts full power, i.e. the extrapolated free-spin maximum. */
    public double vMax() {
      return kV > 1e-9 ? (1.0 - kS) / kV : 0.0;
    }

    /** Open-loop power the model predicts is needed to hold {@code velocity}. */
    public double predictPower(double velocity) {
      return kV * velocity + kS;
    }

    /**
     * Proportional gain giving the requested closed-loop gain. The plant's DC gain is {@code 1/kV}
     * ticks/s per unit power, so a proportional gain of {@code loopGain * kV} yields exactly that
     * loop gain. Values at or above 1.0 oscillate once sampling delay is included; 0.2-0.4 is a
     * sane starting range.
     */
    public double proportionalGainFor(double loopGain) {
      return loopGain * kV;
    }
  }

  /**
   * Fits {@code u = kV * v + kS} over the supplied samples by ordinary least squares.
   *
   * @throws IllegalArgumentException if fewer than two samples are supplied
   */
  public static Result fit(List<Sample> samples) {
    if (samples == null || samples.size() < 2) {
      throw new IllegalArgumentException("Need at least 2 samples to fit a line");
    }

    int n = samples.size();
    double sumV = 0;
    double sumU = 0;
    double sumVV = 0;
    double sumVU = 0;

    for (Sample s : samples) {
      sumV += s.velocityTicksPerSec();
      sumU += s.power();
      sumVV += s.velocityTicksPerSec() * s.velocityTicksPerSec();
      sumVU += s.velocityTicksPerSec() * s.power();
    }

    double denom = n * sumVV - sumV * sumV;
    if (Math.abs(denom) < 1e-9) {
      throw new IllegalArgumentException("Samples are degenerate (all at the same velocity)");
    }

    double kV = (n * sumVU - sumV * sumU) / denom;
    double kS = (sumU - kV * sumV) / n;

    double sumSq = 0.0;
    double maxAbs = 0.0;
    for (Sample s : samples) {
      double residual = s.power() - (kV * s.velocityTicksPerSec() + kS);
      sumSq += residual * residual;
      maxAbs = Math.max(maxAbs, Math.abs(residual));
    }

    return new Result(kV, kS, Math.sqrt(sumSq / n), maxAbs);
  }
}
