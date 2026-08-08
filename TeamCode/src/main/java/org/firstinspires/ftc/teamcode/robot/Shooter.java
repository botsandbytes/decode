package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.AntiWindupIntegrator;

@Configurable
public class Shooter {

  private final DcMotorEx shooter1;
  private final DcMotorEx shooter2;
  private final Servo hood;
  private final VoltageSensor voltageSensor;
  private final PIDFController pidfController;
  private final PIDFCoefficients coefficients;
  private final AntiWindupIntegrator integrator = new AntiWindupIntegrator();
  private final ElapsedTime loopTimer = new ElapsedTime();
  private final ElapsedTime voltageTimer = new ElapsedTime();

  private double targetPower = 0.0;
  private double targetHoodPosition;

  /** Saturation direction of the previous command: +1 railed high, -1 railed low, 0 in range. */
  private int lastSaturationSign = 0;

  // Diagnostics from the most recent periodic() pass, for telemetry only.
  private double lastTargetVelocity = 0.0;
  private double lastError = 0.0;
  private double lastPidTerm = 0.0;
  private double lastIntegralTerm = 0.0;
  private double lastFeedforwardTerm = 0.0;
  private double lastCommand = 0.0;
  private double lastBusVoltage = 0.0;
  private double lastVoltageScale = 1.0;

  public Shooter(HardwareMap hardwareMap) {
    shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
    shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
    hood = hardwareMap.get(Servo.class, "hood");
    voltageSensor = hardwareMap.voltageSensor.iterator().next();

    shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

    shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

    // Pedro's I is held at 0 — the integral term is run by AntiWindupIntegrator instead, because
    // PIDFController accumulates its integral unbounded. See setShooterPIDFCoefficients().
    coefficients =
        new PIDFCoefficients(
            config.shooter.pidf.p, 0.0, config.shooter.pidf.d, config.shooter.pidf.f);
    pidfController = new PIDFController(coefficients);

    this.targetHoodPosition = config.shooter.ballistics.min_hood_servo_pos;
    hood.setPosition(this.targetHoodPosition);
  }

  /**
   * Re-reads the PIDF gains from config so a hot-pushed config.yaml takes effect without an APK
   * rebuild. This mutates the coefficients object in place rather than handing the controller a new
   * one: {@link PIDFController#run()} re-reads its gains from the {@code PIDFCoefficientSupplier}
   * it was constructed with, so a {@code setCoefficients(new ...)} call is silently discarded on
   * the very next run().
   */
  public final void setShooterPIDFCoefficients() {
    // I stays 0 here on purpose; config.shooter.pidf.i feeds the AntiWindupIntegrator in
    // periodic().
    coefficients.setCoefficients(
        config.shooter.pidf.p, 0.0, config.shooter.pidf.d, config.shooter.pidf.f);
  }

  public double getShooterVelocity() {
    return shooter1.getVelocity();
  }

  public double getShooterPower() {
    return shooter1.getPower();
  }

  public void stop() {
    shooter1.setPower(0);
    shooter2.setPower(0);
    if (pidfController != null) {
      pidfController.reset();
    }
    integrator.reset();
    lastSaturationSign = 0;
    // Keeps the first pass after a restart from seeing the whole idle period as one dt.
    loopTimer.reset();
  }

  public void setTargetHoodPosition(double position) {
    // Ordered bounds, not the raw min/max angle positions: the hood linkage is reversed (servo 0.0
    // is the steepest arc), so min_hood_servo_pos is numerically the larger of the two. Clamping
    // with them in field order collapses to a constant and pins the hood at one end of its travel.
    double a = config.shooter.ballistics.min_hood_servo_pos;
    double b = config.shooter.ballistics.max_hood_servo_pos;
    this.targetHoodPosition = Math.clamp(position, Math.min(a, b), Math.max(a, b));
  }

  public double getTargetHoodPosition() {
    return targetHoodPosition;
  }

  /**
   * Position the servo controller reports for the hood, read back from hardware. Diverging from
   * {@link #getTargetHoodPosition()} means the write in {@link #periodic()} is not reaching the
   * controller; agreeing while the hood does not physically move points at power or linkage.
   */
  public double getHoodPositionReadback() {
    return hood.getPosition();
  }

  public void setTargetPower(double power) {
    // A setpoint change invalidates the accumulated steady-state correction for the old setpoint.
    if (Math.abs(power - this.targetPower) > 1e-6) {
      integrator.reset();
    }
    this.targetPower = power;
  }

  public double getTargetPower() {
    return targetPower;
  }

  public void periodic() {
    hood.setPosition(targetHoodPosition);
    if (targetPower <= 0) {
      stop();
    } else {
      double targetVel = config.shooter.max_rpm * targetPower;
      // Close the loop on speed, not signed velocity. The flywheel is unidirectional and every
      // other consumer of this reading already uses the magnitude: ShooterCharacterizationOpMode
      // fits kV/kS against |v|, and ShotController's readiness gate compares |v| to |target|.
      // Using the raw signed value here made this the only sign-sensitive consumer, so whenever
      // setDirection(REVERSE) flipped getVelocity()'s sign the error became target + |v| — a
      // positive-feedback loop that pinned the motor at full power.
      double currentVel = Math.abs(getShooterVelocity());

      pidfController.setTargetPosition(targetVel);
      pidfController.updatePosition(currentVel);
      double pidOutput = pidfController.run();

      double error = targetVel - currentVel;
      double integralTerm =
          integrator.update(
              error,
              loopTimer.seconds(),
              config.shooter.pidf.i,
              config.shooter.integral.band_ticks,
              config.shooter.integral.max_contribution,
              lastSaturationSign);
      loopTimer.reset();

      double ks = config.shooter.ks;
      double ff = (config.shooter.pidf.f * targetVel) / 32767.0;

      // Motor power is a duty cycle, not a torque, so the same command produces less wheel speed
      // as the pack sags. kS and kV were fit in one characterization run at one battery state; by
      // the end of a match the same feedforward lands the flywheel well short, and with P this
      // small (a 0.05 power shortfall needs ~340 ticks/s of standing error to correct) the loop
      // cannot make it up. Rescaling to the measured bus voltage makes the plant look
      // voltage-invariant to everything upstream, which is why it wraps the whole command and not
      // just the feedforward.
      refreshBusVoltage();
      lastVoltageScale =
          voltageCompensationScale(
              config.shooter.nominal_voltage,
              lastBusVoltage,
              config.shooter.max_voltage_compensation);

      double raw =
          (pidOutput + integralTerm + ff + Math.copySign(ks, targetVel)) * lastVoltageScale;
      double command = Math.clamp(raw, -1.0, 1.0);

      // Feed this pass's saturation state back to the integrator on the next pass.
      lastSaturationSign = raw > 1.0 ? 1 : (raw < -1.0 ? -1 : 0);

      lastTargetVelocity = targetVel;
      lastError = error;
      lastPidTerm = pidOutput;
      lastIntegralTerm = integralTerm;
      lastFeedforwardTerm = ff + Math.copySign(ks, targetVel);
      lastCommand = command;

      shooter1.setPower(command);
      shooter2.setPower(command);
    }
  }

  /**
   * Refreshes {@link #lastBusVoltage}, at most once per {@code shooter.voltage_refresh_sec}.
   *
   * <p>The hub's voltage is an ADC command with its own round trip — it is not part of the bulk
   * read cleared in {@code Robot.update()}, so reading it every loop would add a few milliseconds
   * to every tick of a loop this codebase otherwise bulk-caches specifically to keep fast. A pack
   * does not sag meaningfully inside a quarter second, so the stale value is as good as a fresh
   * one. A failed read leaves the field at 0, which {@link #voltageCompensationScale} treats as
   * "unknown" and answers with no compensation.
   */
  private void refreshBusVoltage() {
    if (lastBusVoltage > 0.0 && voltageTimer.seconds() < config.shooter.voltage_refresh_sec) {
      return;
    }
    voltageTimer.reset();
    try {
      lastBusVoltage = voltageSensor.getVoltage();
    } catch (RuntimeException e) {
      lastBusVoltage = 0.0;
    }
  }

  /**
   * Factor that rescales a motor command sized for {@code nominalVolts} so it produces the same
   * mechanical output on a bus actually sitting at {@code measuredVolts}.
   *
   * <p>An unusable reading (non-finite, or non-positive because the sensor read failed) yields 1.0
   * — no compensation. Scaling on a reading that cannot be trusted would rail the flywheel on a
   * sensor fault, so an unknown voltage falls back to the uncompensated command rather than to a
   * guess. A genuinely low reading is bounded by {@code maxScale} instead, since that is a real
   * measurement and deserves as much correction as the clamp allows.
   *
   * @param nominalVolts bus voltage the kS/kV feedforward was characterized at
   * @param measuredVolts live bus voltage
   * @param maxScale upper bound on the returned factor; values below 1.0 are ignored
   */
  public static double voltageCompensationScale(
      double nominalVolts, double measuredVolts, double maxScale) {
    if (!Double.isFinite(nominalVolts) || nominalVolts <= 0.0) {
      return 1.0;
    }
    if (!Double.isFinite(measuredVolts) || measuredVolts <= 0.0) {
      return 1.0;
    }
    double scale = nominalVolts / measuredVolts;
    double limit = maxScale >= 1.0 ? maxScale : Double.MAX_VALUE;
    return Math.min(scale, limit);
  }

  /** Bus voltage read on the most recent {@link #periodic()} pass; 0 if the read failed. */
  public double getLastBusVoltage() {
    return lastBusVoltage;
  }

  /** Voltage-compensation factor applied to the most recent motor command. */
  public double getLastVoltageScale() {
    return lastVoltageScale;
  }

  /** Velocity setpoint (ticks/s) used by the most recent {@link #periodic()} pass. */
  public double getLastTargetVelocity() {
    return lastTargetVelocity;
  }

  /** Signed velocity error (ticks/s) from the most recent {@link #periodic()} pass. */
  public double getLastError() {
    return lastError;
  }

  /** Proportional/integral/derivative contribution to the most recent motor command. */
  public double getLastPidTerm() {
    return lastPidTerm;
  }

  /** Anti-windup integral contribution to the most recent motor command. */
  public double getLastIntegralTerm() {
    return lastIntegralTerm;
  }

  /** Combined feedforward (kV*v + kS) contribution to the most recent motor command. */
  public double getLastFeedforwardTerm() {
    return lastFeedforwardTerm;
  }

  /** Final clamped power written to both flywheel motors on the most recent pass. */
  public double getLastCommand() {
    return lastCommand;
  }

  public Command shootCommand(double power) {
    return Command.build()
        .setStart(() -> setTargetPower(power))
        .setEnd(interrupted -> setTargetPower(0))
        .requiring(this);
  }
}
