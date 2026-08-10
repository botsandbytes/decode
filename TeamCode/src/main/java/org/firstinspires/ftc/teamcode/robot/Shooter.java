package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
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

    DcMotor.RunMode initialMode =
        config.shooter.use_ftc_pid
            ? DcMotor.RunMode.RUN_USING_ENCODER
            : DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter1.setMode(initialMode);
    shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

    shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter2.setMode(initialMode);
    shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

    coefficients =
        new PIDFCoefficients(
            config.shooter.pidf.p, 0.0, config.shooter.pidf.d, config.shooter.pidf.f);
    pidfController = new PIDFController(coefficients);

    setShooterPIDFCoefficients();

    this.targetHoodPosition = config.shooter.ballistics.min_hood_servo_pos;
    hood.setPosition(this.targetHoodPosition);
  }

  public final void setShooterPIDFCoefficients() {
    coefficients.setCoefficients(
        config.shooter.pidf.p, 0.0, config.shooter.pidf.d, config.shooter.pidf.f);

    if (config.shooter.use_ftc_pid) {
      shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.shooter.motor_pidf);
      shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.shooter.motor_pidf);
    }
  }

  public double getShooterVelocity() {
    return shooter1.getVelocity();
  }

  public void stop() {
    shooter1.setPower(0);
    shooter2.setPower(0);
    if (config.shooter.use_ftc_pid) {
      shooter1.setVelocity(0);
      shooter2.setVelocity(0);
    }
    if (pidfController != null) {
      pidfController.reset();
    }
    integrator.reset();
    lastSaturationSign = 0;
    loopTimer.reset();
  }

  public void setTargetHoodPosition(double position) {
    double a = config.shooter.ballistics.min_hood_servo_pos;
    double b = config.shooter.ballistics.max_hood_servo_pos;
    this.targetHoodPosition = Math.clamp(position, Math.min(a, b), Math.max(a, b));
  }

  public double getTargetHoodPosition() {
    return targetHoodPosition;
  }

  public double getHoodPositionReadback() {
    return hood.getPosition();
  }

  public static double constantPower() {
    return config.shooter.constant_rpm / config.shooter.max_rpm;
  }

  public void setTargetPower(double power) {
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
      double currentVel = Math.abs(getShooterVelocity());

      if (config.shooter.use_ftc_pid) {
        if (shooter1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
          shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        shooter1.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);

        lastTargetVelocity = targetVel;
        lastError = targetVel - currentVel;
        lastPidTerm = 0.0;
        lastIntegralTerm = 0.0;
        lastFeedforwardTerm = 0.0;
        lastCommand = targetPower;
      } else {
        if (shooter1.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
          shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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

        refreshBusVoltage();
        lastVoltageScale =
            voltageCompensationScale(
                config.shooter.nominal_voltage,
                lastBusVoltage,
                config.shooter.max_voltage_compensation);

        double raw =
            (pidOutput + integralTerm + ff + Math.copySign(ks, targetVel)) * lastVoltageScale;
        double command = Math.clamp(raw, -1.0, 1.0);

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
  }

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

  public double getLastBusVoltage() {
    return lastBusVoltage;
  }

  public double getLastVoltageScale() {
    return lastVoltageScale;
  }

  public double getLastTargetVelocity() {
    return lastTargetVelocity;
  }

  public double getLastError() {
    return lastError;
  }

  public double getLastPidTerm() {
    return lastPidTerm;
  }

  public double getLastIntegralTerm() {
    return lastIntegralTerm;
  }

  public double getLastFeedforwardTerm() {
    return lastFeedforwardTerm;
  }

  public double getLastCommand() {
    return lastCommand;
  }
}
