package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

public class Turret {
  public static final double OFFSET_CONST = 260.0;
  public static final double LIMIT_CONST = 160.0;
  public static final double FAULT_MARGIN_DEG = 10.0;

  private double p;
  private double i;
  private double d;
  private double f;
  private double ksPositive;
  private double ksNegative;
  private double maxPower;

  private final CRServo turnServo;
  private final AnalogInput turnAnalog;
  private final Telemetry telemetry;

  private final PIDFCoefficients pidfCoefficients;
  private final PIDFController pidfController;

  private double targetTurnAngle = 0;
  private boolean isTurnDone = false;
  private double goalX;
  private double goalY;
  private final java.util.function.Supplier<Pose> poseSupplier;

  public enum AimMode {
    IDLE,
    HOLD,
    AIM_AT_GOAL,
    MANUAL
  }

  private AimMode mode = AimMode.IDLE;
  private double holdAngle = 0.0;
  private double manualPower = 0.0;

  public Turret(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
    this(hardwareMap, telemetry, follower::getPose);
  }

  public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
    this(hardwareMap, telemetry, () -> new Pose(0, 0, 0));
  }

  public Turret(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      java.util.function.Supplier<Pose> poseSupplier) {
    this.telemetry = telemetry;
    this.poseSupplier = poseSupplier;

    turnServo = hardwareMap.get(CRServo.class, "turn");
    turnAnalog = hardwareMap.get(AnalogInput.class, "turnanalog");

    if (turnServo != null && config.turret != null) {
      boolean flip = config.turret.servo_direction_inverted;
      turnServo.setDirection(flip ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
    }

    reloadFromConfig();

    pidfCoefficients = new PIDFCoefficients(p, i, d, f);
    pidfController = new PIDFController(pidfCoefficients);

    if (config.turret != null && config.turret.analog_encoder != null) {
      this.zeroVoltageOffset = config.turret.analog_encoder.zero_voltage;
    }
  }

  public final void reloadFromConfig() {
    if (config.turret != null) {
      if (config.turret.pidf != null) {
        this.p = config.turret.pidf.p;
        this.i = config.turret.pidf.i;
        this.d = config.turret.pidf.d;
        this.f = config.turret.pidf.f;
      }
      this.ksPositive = config.turret.ks_positive;
      this.ksNegative = config.turret.ks_negative;
      this.maxPower = config.turret.max_power_output;
    }
    // Null during the constructor's own call, before the coefficients exist.
    if (pidfCoefficients != null) {
      applyPIDFCoefficients();
    }
  }

  public void setPIDF(double p, double i, double d, double f) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
    applyPIDFCoefficients();
  }

  public void setKs(double ksPositive, double ksNegative) {
    this.ksPositive = ksPositive;
    this.ksNegative = ksNegative;
  }

  public double getKsPositive() {
    return ksPositive;
  }

  public double getKsNegative() {
    return ksNegative;
  }

  public void setMaxPower(double maxPower) {
    this.maxPower = maxPower;
  }

  public double getP() {
    return p;
  }

  public double getI() {
    return i;
  }

  public double getD() {
    return d;
  }

  public double getF() {
    return f;
  }

  public double getMaxPower() {
    return maxPower;
  }

  private void applyPIDFCoefficients() {
    pidfCoefficients.setCoefficients(p, i, d, f);
  }

  private double zeroVoltageOffset = 0.0;

  public void zeroAtCurrentPosition() {
    if (turnAnalog != null) {
      this.zeroVoltageOffset = turnAnalog.getVoltage();
    }
  }

  public double getZeroVoltage() {
    return (zeroVoltageOffset != 0.0)
        ? zeroVoltageOffset
        : (config.turret != null && config.turret.analog_encoder != null
            ? config.turret.analog_encoder.zero_voltage
            : 0.0);
  }

  public void setGoal(double x, double y) {
    this.goalX = x;
    this.goalY = y;
  }

  public void setTargetTurnAngle(double degrees) {
    var travel = config.turret.travel;
    this.targetTurnAngle = Math.clamp(degrees, travel.min_angle, travel.max_angle);
  }

  public void setHoldAngle(double angle) {
    var travel = config.turret.travel;
    this.holdAngle = Math.clamp(angle, travel.min_angle, travel.max_angle);
  }

  public double getHoldAngle() {
    return holdAngle;
  }

  public double getRawVoltage() {
    if (turnAnalog != null && config.turret.analog_encoder.enabled) {
      return turnAnalog.getVoltage();
    }
    return Double.NaN;
  }

  public boolean isReadingWithinTravel() {
    if (Double.isNaN(getRawVoltage())) {
      return true;
    }
    var travel = config.turret.travel;
    double angle = getCurrentTurnAngle();
    return angle >= travel.min_angle - FAULT_MARGIN_DEG
        && angle <= travel.max_angle + FAULT_MARGIN_DEG;
  }

  public double getCurrentTurnAngle() {
    if (turnAnalog != null && config.turret.analog_encoder.enabled) {
      return angleForVoltage(turnAnalog.getVoltage());
    }
    return 0.0;
  }

  public double angleForVoltage(double voltage) {
    var enc = config.turret.analog_encoder;
    double delta = voltage - getZeroVoltage();
    double fullScale = enc.full_scale_voltage;

    if (fullScale > 0) {
      double half = fullScale / 2.0;
      delta -= Math.floor((delta + half) / fullScale) * fullScale;
    }

    if (enc.inverted) {
      delta = -delta;
    }
    return delta * enc.degrees_per_volt;
  }

  public double[] getReachableAngleRange() {
    var enc = config.turret.analog_encoder;
    double atMin = angleForVoltage(enc.min_voltage);
    double atMax = angleForVoltage(enc.max_voltage);
    return new double[] {Math.min(atMin, atMax), Math.max(atMin, atMax)};
  }

  public String getEncoderRangeWarning() {
    var travel = config.turret.travel;
    var enc = config.turret.analog_encoder;
    double[] range = getReachableAngleRange();
    StringBuilder sb = new StringBuilder();

    double expectedSpan = (enc.max_voltage - enc.min_voltage) * enc.degrees_per_volt;
    double actualSpan = range[1] - range[0];
    if (enc.full_scale_voltage > 0 && Math.abs(actualSpan - expectedSpan) > 1.0) {
      sb.append(
          String.format(
              Locale.ROOT,
              "zero_voltage %.4f puts the encoder wrap inside the turret's travel "
                  + "(stops span %.1f deg, expected %.1f deg) - angle will jump mid-travel. ",
              getZeroVoltage(),
              actualSpan,
              expectedSpan));
    }

    if (range[1] < travel.max_angle) {
      sb.append(
          String.format(
              Locale.ROOT,
              "max_angle %.1f unreachable (encoder tops out at %.1f). ",
              travel.max_angle,
              range[1]));
    }
    if (range[0] > travel.min_angle) {
      sb.append(
          String.format(
              Locale.ROOT,
              "min_angle %.1f unreachable (encoder bottoms out at %.1f). ",
              travel.min_angle,
              range[0]));
    }
    return sb.isEmpty() ? null : sb.toString().trim();
  }

  private Boolean enabledOverride = null;

  public boolean isEnabled() {
    if (enabledOverride != null) {
      return enabledOverride;
    }
    return config.turret != null && config.turret.enabled;
  }

  public void setEnabled(boolean enabled) {
    this.enabledOverride = enabled;
  }

  public boolean isTurnDone() {
    if (!isEnabled()) {
      return true;
    }
    return isTurnDone;
  }

  private boolean wouldExceedTurnBoundary(double power) {
    if (power == 0) return false;
    var travel = config.turret.travel;
    double angle = getCurrentTurnAngle();
    return (power > 0 && angle >= travel.max_angle) || (power < 0 && angle <= travel.min_angle);
  }

  public String getPowerBlockedReason() {
    return powerBlockedReason;
  }

  private String powerBlockedReason = null;

  public void clearFault() {
    faultReason = null;
    stallReferenceAngle = Double.NaN;
    runawayTarget = Double.NaN;
  }

  public boolean isFaulted() {
    return faultReason != null;
  }

  public String getFaultReason() {
    return faultReason;
  }

  private String faultReason = null;
  private double stallReferenceAngle = Double.NaN;
  private long stallReferenceTimeNano = 0L;
  private double runawayTarget = Double.NaN;
  private double runawayBestAbsError = Double.MAX_VALUE;

  private void updateRunawayWatchdog(double target, double error, double command) {
    var runaway = config.turret.runaway;
    if (!runaway.enabled || Double.isNaN(getRawVoltage()) || isFaulted()) {
      return;
    }

    if (Double.isNaN(runawayTarget) || Math.abs(target - runawayTarget) > 1e-6) {
      runawayTarget = target;
      runawayBestAbsError = Math.abs(error);
      return;
    }

    if (Math.abs(command) < config.turret.stall.power_threshold) {
      return;
    }

    double absError = Math.abs(error);
    runawayBestAbsError = Math.min(runawayBestAbsError, absError);
    if (absError > runawayBestAbsError + runaway.divergence_deg) {
      faultReason =
          String.format(
              Locale.ROOT,
              "RUNAWAY - error grew %.1f -> %.1f deg under power. The loop is driving away from "
                  + "its target: check servo_direction_inverted vs analog_encoder.inverted.",
              runawayBestAbsError,
              absError);
    }
  }

  private void updateStallWatchdog(double commandedPower) {
    var stall = config.turret.stall;
    if (!stall.enabled || Double.isNaN(getRawVoltage()) || isFaulted()) {
      return;
    }

    long now = System.nanoTime();
    double angle = getCurrentTurnAngle();

    if (Math.abs(commandedPower) < stall.power_threshold) {
      stallReferenceAngle = Double.NaN;
      return;
    }

    if (Double.isNaN(stallReferenceAngle)) {
      stallReferenceAngle = angle;
      stallReferenceTimeNano = now;
      return;
    }

    if (Math.abs(angle - stallReferenceAngle) >= stall.min_progress_deg) {
      stallReferenceAngle = angle;
      stallReferenceTimeNano = now;
      return;
    }

    if ((now - stallReferenceTimeNano) / 1e9 >= stall.timeout_sec) {
      faultReason =
          String.format(
              Locale.ROOT,
              "STALLED at %.1f deg - power commanded but the turret is not moving.",
              getCurrentTurnAngle());
    }
  }

  public void setTurretPowerRaw(double power) {
    powerBlockedReason = null;

    if (!isEnabled()) {
      powerBlockedReason = "TURRET DISABLED";
      power = 0;
    } else {
      updateStallWatchdog(power);

      if (isFaulted()) {
        powerBlockedReason = faultReason;
        power = 0;
      } else if (!isReadingWithinTravel()) {
        powerBlockedReason =
            String.format(
                Locale.ROOT,
                "ENCODER OUT OF RANGE (%.1f deg) - reading is not trusted, power cut",
                getCurrentTurnAngle());
        power = 0;
      } else if (wouldExceedTurnBoundary(power)) {
        powerBlockedReason = "AT TRAVEL LIMIT";
        power = 0;
      }
    }

    if (turnServo != null) {
      boolean flip = config.turret != null && config.turret.servo_direction_inverted;
      CRServo.Direction targetDir = flip ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD;
      if (turnServo.getDirection() != targetDir) {
        turnServo.setDirection(targetDir);
      }
      turnServo.setPower(power);
    }
  }

  public static Pose alignPose(double x, double y, double goalX, double goalY) {
    double deltaX = goalX - x;
    double deltaY = goalY - y;
    double angleRadians = Math.atan2(deltaY, deltaX);
    return new Pose(x, y, angleRadians);
  }

  public double calculateDynamicTolerance(double distanceInches) {
    var tol = config.turret.tolerance;
    if (distanceInches < tol.near_cutoff) return tol.near_val;
    double ratio = Math.clamp((tol.max_drift / 2.0) / distanceInches, -1.0, 1.0);
    double thetaRadians = Math.asin(ratio);
    return Math.clamp(Math.toDegrees(thetaRadians), tol.min_deg, tol.max_deg);
  }

  public boolean shouldTurnLeft(double current, double target) {
    double cR = (current + OFFSET_CONST) % 360;
    double tR = (target + OFFSET_CONST) % 360;

    boolean lBad = cR <= tR ? (cR <= LIMIT_CONST && tR >= 0) : (cR <= LIMIT_CONST || tR >= 0);
    boolean rBad = tR <= cR ? (tR <= LIMIT_CONST && cR >= 0) : (tR <= LIMIT_CONST || tR >= 0);

    double ld = (target - current + 360) % 360;
    double rd = (current - target + 360) % 360;

    return lBad == rBad ? (ld <= rd) : !lBad;
  }

  public static double computeSignedError(
      double relativeCurrent, double relativeTarget, boolean turnLeft) {
    double error;
    if (turnLeft) {
      error = (relativeTarget - relativeCurrent + 360) % 360;
      if (error > 180) error -= 360;
    } else {
      error = -((relativeCurrent - relativeTarget + 360) % 360);
      if (error < -180) error += 360;
    }
    return error;
  }

  public void updateTurret(Pose currentPose) {
    double relativeTurretAngle = getCurrentTurnAngle();
    double relativeTargetAngle = targetTurnAngle;

    double error = AngleUnit.normalizeDegrees(relativeTargetAngle - relativeTurretAngle);
    boolean turnLeft = error > 0;
    double tolerance = calculateDynamicTolerance(distanceToGoal(currentPose));
    double absError = Math.abs(error);

    double feedforward = 0.0;
    if (ksPositive != 0.0 || ksNegative != 0.0) {
      if (error > 0) {
        feedforward = Math.abs(ksPositive);
      } else if (error < 0) {
        feedforward = -Math.abs(ksNegative);
      }
    }

    applyPIDFCoefficients();
    pidfController.setTargetPosition(relativeTargetAngle);
    pidfController.updatePosition(relativeTurretAngle);
    pidfController.updateFeedForwardInput(Math.signum(error));
    double pidOutput = pidfController.run();
    double command = Math.clamp(pidOutput + feedforward, -maxPower, maxPower);

    updateRunawayWatchdog(relativeTargetAngle, error, command);

    boolean isBoundaryViolated = wouldExceedTurnBoundary(command);

    if (absError <= tolerance || isBoundaryViolated) {
      setTurretPowerRaw(0);
      isTurnDone = absError <= tolerance;
      pidfController.reset();
    } else {
      isTurnDone = false;
      setTurretPowerRaw(command);
    }

    if (telemetry != null) {
      telemetry.addData("Robot Heading", Math.toDegrees(currentPose.getHeading()));
      telemetry.addData("Target Relative", relativeTargetAngle);
      telemetry.addData("Turret Relative", relativeTurretAngle);
      telemetry.addData("Turret Power", turnServo != null ? turnServo.getPower() : 0.0);
      telemetry.addData("Turret Error", error);
      telemetry.addData("Turret Tolerance", tolerance);
      telemetry.addData("Turret Done", isTurnDone);
      telemetry.addData("Turret Direction", turnLeft ? "LEFT" : "RIGHT");
      telemetry.addData("Turret At Limit", isBoundaryViolated);
      telemetry.addData("Turret Encoder Fault", !isReadingWithinTravel());
      telemetry.addData(
          "Turret Power Blocked", powerBlockedReason == null ? "no" : powerBlockedReason);
      telemetry.addData("Encoder Raw Voltage", getRawVoltage());
      telemetry.addData("Encoder Live Angle", getCurrentTurnAngle());
    }
  }

  private double distanceToGoal(Pose currentPose) {
    return Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
  }

  public double getTargetTurnAngle() {
    return targetTurnAngle;
  }

  public double getAimError(Pose currentPose) {
    double relativeTurretAngle = getCurrentTurnAngle();
    double relativeTargetAngle = targetTurnAngle;

    return AngleUnit.normalizeDegrees(relativeTargetAngle - relativeTurretAngle);
  }

  public boolean isAimed(Pose currentPose) {
    if (!isEnabled()) {
      return true;
    }
    double error = getAimError(currentPose);
    double dynamicTolerance = calculateDynamicTolerance(distanceToGoal(currentPose));
    return Math.abs(error) < dynamicTolerance;
  }

  public void setAimMode(AimMode mode) {
    this.mode = mode;
  }

  public AimMode getAimMode() {
    return mode;
  }

  private double targetAzimuthRad = Double.NaN;

  public void setTargetAzimuth(double azimuthRad) {
    this.targetAzimuthRad = azimuthRad;
  }

  public void setManualPower(double power) {
    this.manualPower = power;
  }

  public void periodic() {
    if (!isEnabled()) {
      setTurretPowerRaw(0);
      return;
    }
    switch (mode) {
      case IDLE:
        setTurretPowerRaw(0);
        break;
      case HOLD:
        this.targetTurnAngle = holdAngle;
        updateTurret(poseSupplier.get());
        break;
      case AIM_AT_GOAL:
        Pose pose = poseSupplier.get();
        if (pose != null) {
          double worldBearingDegrees;
          if (!Double.isNaN(targetAzimuthRad)) {
            worldBearingDegrees = Math.toDegrees(targetAzimuthRad);
          } else {
            double deltaX = goalX - pose.getX();
            double deltaY = goalY - pose.getY();
            worldBearingDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));
          }
          double robotWorldHeadingDegrees = Math.toDegrees(pose.getHeading());
          setTargetTurnAngle(
              AngleUnit.normalizeDegrees(worldBearingDegrees - robotWorldHeadingDegrees));
          updateTurret(pose);
        }
        break;
      case MANUAL:
        setTurretPowerRaw(manualPower);
        break;
    }
  }
}
