package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@Configurable
public class Turret {
  public static final double OFFSET_CONST = 260.0;
  public static final double LIMIT_CONST = 160.0;
  public static final double MAX_JUMP_DEG = 30.0;
  public static final double FAULT_MARGIN_DEG = 10.0;
  public static final double RUNAWAY_DEG = 8.0;

  private final CRServo turnServo;
  private final AnalogInput turnAnalog;
  private final Telemetry telemetry;

  private final PIDFController pidfController;

  private double targetTurnAngle = 0;
  private boolean isTurnDone = false;
  private boolean aimedLatch = false;
  private double lastEncoderAngle = 0.0;
  private boolean hasLastEncoderAngle = false;
  private boolean encoderFaultLatched = false;
  private boolean runawayFaultLatched = false;
  private double runawayRefAngle = 0.0;
  private double runawayCommandSign = 0.0;
  private double lastCommandedPower = 0.0;
  private final ElapsedTime stallTimer = new ElapsedTime();
  private double stallReferenceAngle = Double.NaN;
  private boolean stallKickActive = false;
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

    var p = config.turret.pidf;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p.p, p.i, p.d, p.f);
    pidfController = new PIDFController(pidfCoefficients);

    if (config.turret != null && config.turret.analog_encoder != null) {
      this.zeroVoltageOffset = config.turret.analog_encoder.zero_voltage;
    }
  }

  private double zeroVoltageOffset = 0.0;

  public void zeroAtCurrentPosition() {
    if (turnAnalog != null) {
      this.zeroVoltageOffset = turnAnalog.getVoltage();
    }
  }

  public void setZeroVoltage(double voltage) {
    this.zeroVoltageOffset = voltage;
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

  /** Raw analog encoder voltage, or NaN when the encoder is absent or disabled. */
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

  public boolean isEncoderFaulted() {
    return encoderFaultLatched;
  }

  public boolean isRunawayFaulted() {
    return runawayFaultLatched;
  }

  public boolean isStallKickActive() {
    return stallKickActive;
  }

  public void clearEncoderFault() {
    encoderFaultLatched = false;
    runawayFaultLatched = false;
    hasLastEncoderAngle = false;
    runawayCommandSign = 0.0;
  }

  private void updateRunawayDetection(double angle, double command) {
    double sign = Math.signum(command);
    if (sign == 0.0) {
      runawayCommandSign = 0.0;
      return;
    }
    if (sign != runawayCommandSign) {
      runawayCommandSign = sign;
      runawayRefAngle = angle;
      return;
    }
    double progress = (angle - runawayRefAngle) * sign;
    if (progress < -RUNAWAY_DEG) {
      runawayFaultLatched = true;
    } else if (progress > 0) {
      runawayRefAngle = angle;
    }
  }

  private void resetStallDetection() {
    stallReferenceAngle = Double.NaN;
    stallKickActive = false;
  }

  private double applyStallKick(double command, double angle, double error) {
    var stall = config.turret.stall;

    if (Double.isNaN(stallReferenceAngle)
        || Math.abs(angle - stallReferenceAngle) >= stall.progress_deg) {
      stallReferenceAngle = angle;
      stallTimer.reset();
      stallKickActive = false;
      return command;
    }

    if (!stallKickActive && stallTimer.seconds() < stall.timeout_sec) {
      return command;
    }

    stallKickActive = true;
    return Math.copySign(Math.max(Math.abs(command), stall.kick_power), error);
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
    double[] range = getReachableAngleRange();
    StringBuilder sb = new StringBuilder();
    if (range[1] < travel.max_angle) {
      sb.append(
          String.format(
              "max_angle %.1f unreachable (encoder tops out at %.1f). ",
              travel.max_angle, range[1]));
    }
    if (range[0] > travel.min_angle) {
      sb.append(
          String.format(
              "min_angle %.1f unreachable (encoder bottoms out at %.1f). ",
              travel.min_angle, range[0]));
    }
    return sb.length() == 0 ? null : sb.toString().trim();
  }

  public boolean isTurnDone() {
    return isTurnDone;
  }

  private boolean wouldExceedTurnBoundary(boolean increasingAngle) {
    var travel = config.turret.travel;
    double angle = getCurrentTurnAngle();
    return (angle >= travel.max_angle && increasingAngle)
        || (angle <= travel.min_angle && !increasingAngle);
  }

  public void setTurretPowerRaw(double power) {
    if (power > 0 && wouldExceedTurnBoundary(true)) {
      power = 0;
    } else if (power < 0 && wouldExceedTurnBoundary(false)) {
      power = 0;
    }

    boolean flip = config.turret.servo_direction_inverted;
    CRServo.Direction increasing = flip ? CRServo.Direction.FORWARD : CRServo.Direction.REVERSE;
    CRServo.Direction decreasing = flip ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD;

    if (power > 0) {
      turnServo.setDirection(increasing);
      turnServo.setPower(Math.abs(power));
    } else if (power < 0) {
      turnServo.setDirection(decreasing);
      turnServo.setPower(Math.abs(power));
    } else {
      turnServo.setPower(0);
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

    var t = config.turret;

    if (!isReadingWithinTravel()) {
      encoderFaultLatched = true;
    }

    boolean jumpRejected =
        hasLastEncoderAngle && Math.abs(relativeTurretAngle - lastEncoderAngle) > MAX_JUMP_DEG;
    double controlAngle = jumpRejected ? lastEncoderAngle : relativeTurretAngle;
    lastEncoderAngle = relativeTurretAngle;
    hasLastEncoderAngle = true;
    relativeTurretAngle = controlAngle;

    updateRunawayDetection(relativeTurretAngle, lastCommandedPower);

    boolean encoderFault = encoderFaultLatched || runawayFaultLatched || jumpRejected;

    double error = AngleUnit.normalizeDegrees(relativeTargetAngle - relativeTurretAngle);
    boolean turnLeft = error > 0;

    double tolerance = calculateDynamicTolerance(distanceToGoal(currentPose));
    double absError = Math.abs(error);
    if (aimedLatch) {
      if (absError > tolerance * t.aim_hysteresis) aimedLatch = false;
    } else {
      if (absError < tolerance) aimedLatch = true;
    }

    boolean isBoundaryViolated = wouldExceedTurnBoundary(turnLeft);

    if (encoderFault || aimedLatch || isBoundaryViolated) {
      turnServo.setPower(0);
      lastCommandedPower = 0.0;
      isTurnDone = aimedLatch;
      pidfController.reset();
      resetStallDetection();
    } else {
      isTurnDone = false;

      pidfController.setTargetPosition(relativeTargetAngle);
      pidfController.updatePosition(relativeTurretAngle);
      pidfController.updateFeedForwardInput(Math.signum(error));
      double pidOutput = pidfController.run();

      double command = pidOutput + Math.copySign(t.ks, error);
      command = Math.clamp(command, -t.max_power_output, t.max_power_output);

      command = applyStallKick(command, relativeTurretAngle, error);
      command = Math.clamp(command, -t.max_power_output, t.max_power_output);

      setTurretPowerRaw(command);
      lastCommandedPower = command;
    }

    String rangeWarning = getEncoderRangeWarning();
    telemetry.addData("Robot Heading", Math.toDegrees(currentPose.getHeading()));
    telemetry.addData("Target Relative", relativeTargetAngle);
    telemetry.addData("Turret Relative", relativeTurretAngle);
    telemetry.addData("Turret Power", turnServo.getPower());
    telemetry.addData("Turret Error", error);
    telemetry.addData("Turret Tolerance", tolerance);
    telemetry.addData("Turret Done", isTurnDone);
    telemetry.addData("Turret Direction", turnLeft ? "LEFT" : "RIGHT");
    telemetry.addData("Turret At Limit", isBoundaryViolated);
    telemetry.addData("Turret Stall Kick", stallKickActive);
    telemetry.addData("Encoder Raw Voltage", getRawVoltage());
    telemetry.addData("Encoder Live Angle", getCurrentTurnAngle());
    telemetry.addData("Encoder Jump Rejected", jumpRejected);
    if (runawayFaultLatched) {
      telemetry.addLine(
          "RUNAWAY FAULT: turret moved opposite to command -- turret disabled."
              + " Flip turret.servo_direction_inverted (NOT analog_encoder.inverted).");
    }
    if (encoderFaultLatched) {
      telemetry.addData(
          "ENCODER FAULT",
          String.format(
              "voltage %.4f outside calibrated [%.4f, %.4f] -- turret disabled, re-home and"
                  + " re-calibrate",
              getRawVoltage(),
              config.turret.analog_encoder.min_voltage,
              config.turret.analog_encoder.max_voltage));
    }
    if (rangeWarning != null) {
      telemetry.addData("ENCODER RANGE WARNING", rangeWarning);
    }
  }

  private double distanceToGoal(Pose currentPose) {
    return Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
  }

  public void driveTurretRaw(double power) {
    turnServo.setPower(power);
  }

  public double getGoalX() {
    return goalX;
  }

  public double getGoalY() {
    return goalY;
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

  public void setManualPower(double power) {
    this.manualPower = power;
  }

  public void periodic() {
    switch (mode) {
      case IDLE:
        setTurretPowerRaw(0);
        resetStallDetection();
        break;
      case HOLD:
        this.targetTurnAngle = holdAngle;
        updateTurret(poseSupplier.get());
        break;
      case AIM_AT_GOAL:
        Pose pose = poseSupplier.get();
        if (pose != null) {
          double deltaX = goalX - pose.getX();
          double deltaY = goalY - pose.getY();
          double worldBearingDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));
          double robotWorldHeadingDegrees = Math.toDegrees(pose.getHeading());
          setTargetTurnAngle(
              AngleUnit.normalizeDegrees(worldBearingDegrees - robotWorldHeadingDegrees));
          updateTurret(pose);
        }
        break;
      case MANUAL:
        setTurretPowerRaw(manualPower);
        resetStallDetection();
        break;
    }
  }
}
