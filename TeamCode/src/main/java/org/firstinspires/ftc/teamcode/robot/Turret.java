package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.config.config;

@Configurable
public class Turret {
  private final CachingCRServo turnServo;
  private final IMU turnIMU;
  private final Telemetry telemetry;

  private final CachingDcMotorEx leftFront;
  private final CachingDcMotorEx leftBack;
  private final CachingDcMotorEx rightFront;
  private final CachingDcMotorEx rightBack;

  private final PIDFController headingController;
  private final PIDFController pidfController;

  private double targetTurnAngle = 90;
  private double initialHeadingOffset = 0;
  private boolean isTurnDone = false;
  private double goalX;
  private double goalY;
  private final java.util.function.Supplier<Pose> poseSupplier;

  public enum AimMode {
    IDLE,
    HOLD,
    AIM_AT_GOAL
  }

  private AimMode mode = AimMode.IDLE;
  private double holdAngle = 0.0;

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

    turnServo = new CachingCRServo(hardwareMap.get(CRServo.class, "turn"));
    turnIMU = hardwareMap.get(IMU.class, "turnImu");

    DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "leftFront");
    DcMotorEx lb = hardwareMap.get(DcMotorEx.class, "leftBack");
    DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rightFront");
    DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "rightBack");

    leftFront = (lf instanceof CachingDcMotorEx) ? (CachingDcMotorEx) lf : new CachingDcMotorEx(lf);
    leftBack = (lb instanceof CachingDcMotorEx) ? (CachingDcMotorEx) lb : new CachingDcMotorEx(lb);
    rightFront =
        (rf instanceof CachingDcMotorEx) ? (CachingDcMotorEx) rf : new CachingDcMotorEx(rf);
    rightBack = (rb instanceof CachingDcMotorEx) ? (CachingDcMotorEx) rb : new CachingDcMotorEx(rb);

    double tol = config.caching.drivetrain_tolerance;
    leftFront.setCachingTolerance(tol);
    leftBack.setCachingTolerance(tol);
    rightFront.setCachingTolerance(tol);
    rightBack.setCachingTolerance(tol);

    String logoStr = config.turret.orientation.logo;
    String usbStr = config.turret.orientation.usb;

    RevHubOrientationOnRobot.LogoFacingDirection logo;
    try {
      logo =
          !logoStr.isEmpty()
              ? RevHubOrientationOnRobot.LogoFacingDirection.valueOf(logoStr)
              : RevHubOrientationOnRobot.LogoFacingDirection.UP;
    } catch (IllegalArgumentException e) {
      logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    }

    RevHubOrientationOnRobot.UsbFacingDirection usb;
    try {
      usb =
          !usbStr.isEmpty()
              ? RevHubOrientationOnRobot.UsbFacingDirection.valueOf(usbStr)
              : RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    } catch (IllegalArgumentException e) {
      usb = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    }

    turnIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
    turnIMU.resetYaw();

    PIDFCoefficients headingPIDFCoefficients =
        Constants.followerConstants.getCoefficientsHeadingPIDF();
    headingController = new PIDFController(headingPIDFCoefficients);

    var p = config.turret.pidf;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p.p, p.i, p.d, p.f);
    pidfController = new PIDFController(pidfCoefficients);
  }

  public void setGoal(double x, double y) {
    this.goalX = x;
    this.goalY = y;
  }

  public void setInitialHeading(double heading) {
    this.initialHeadingOffset = heading;
  }

  public void setTargetTurnAngle(double degrees) {
    this.targetTurnAngle = degrees;
  }

  public double getCurrentTurnAngle() {
    YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
    return Math.toDegrees(orientation.getYaw(AngleUnit.RADIANS) + initialHeadingOffset);
  }

  public boolean isTurnDone() {
    return isTurnDone;
  }

  public void setTurretPowerRaw(double power) {
    if (power > 0) {
      turnServo.setDirection(CRServo.Direction.FORWARD);
      turnServo.setPower(Math.abs(power));
    } else if (power < 0) {
      turnServo.setDirection(CRServo.Direction.REVERSE);
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
    var turn = config.turret.turn;
    double cR = (current + turn.offset_const) % 360;
    double tR = (target + turn.offset_const) % 360;

    boolean lBad =
        cR <= tR ? (cR <= turn.limit_const && tR >= 0) : (cR <= turn.limit_const || tR >= 0);
    boolean rBad =
        tR <= cR ? (tR <= turn.limit_const && cR >= 0) : (tR <= turn.limit_const || cR >= 0);

    double ld = (target - current + 360) % 360;
    double rd = (current - target + 360) % 360;

    return lBad == rBad ? (ld <= rd) : !lBad;
  }

  /**
   * Signed wrapped angle error to turn from {@code relativeCurrent} to {@code relativeTarget} in
   * the direction given by {@code turnLeft} (as decided by {@link #shouldTurnLeft}).
   */
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
    double robotWorldHeading = currentPose.getHeading();
    double turretWorldAngle = getCurrentTurnAngle();
    double targetWorldAngle = targetTurnAngle;

    double relativeTurretAngle = (turretWorldAngle - robotWorldHeading + 360) % 360;
    double relativeTargetAngle = (targetWorldAngle - robotWorldHeading + 360) % 360;

    boolean turnLeft = shouldTurnLeft(relativeTurretAngle, relativeTargetAngle);
    double error = getAimError(currentPose);

    if (isAimed(currentPose)) {
      turnServo.setPower(0);
      isTurnDone = true;
      pidfController.reset();
    } else {
      isTurnDone = false;

      pidfController.setTargetPosition(relativeTurretAngle + error);
      pidfController.updatePosition(relativeTurretAngle);
      pidfController.updateFeedForwardInput(1);
      double power = pidfController.run();

      var t = config.turret;
      double absPower =
          (power == 0) ? 0 : Math.clamp(Math.abs(power), t.feed_forward, t.max_power_output);

      if (turnLeft) {
        turnServo.setDirection(CRServo.Direction.FORWARD);
      } else {
        turnServo.setDirection(CRServo.Direction.REVERSE);
      }
      turnServo.setPower(absPower);
    }

    telemetry.addData("Turret World", turretWorldAngle);
    telemetry.addData("Turret Target", targetWorldAngle);
    telemetry.addData("Robot Heading", robotWorldHeading);
    telemetry.addData("Target Relative", relativeTargetAngle);
    telemetry.addData("Turret Relative", relativeTurretAngle);
    telemetry.addData("Turret Power", turnServo.getPower());
    telemetry.addData("Turret Error", error);
    telemetry.addData("Turret Done", isTurnDone);
    telemetry.addData("Turret Direction", turnLeft ? "LEFT" : "RIGHT");
  }

  public boolean updateTurn(Pose currentPose, double targetHeadingDegrees) {
    double currentHeadingRad = currentPose.getHeading();
    double targetHeadingRad = Math.toRadians(targetHeadingDegrees);

    double errorAbs = MathFunctions.getSmallestAngleDifference(currentHeadingRad, targetHeadingRad);
    double direction = MathFunctions.getTurnDirection(currentHeadingRad, targetHeadingRad);
    double signedError = errorAbs * direction;

    headingController.updateFeedForwardInput(direction);
    headingController.updateError(signedError);

    double pidOutput = headingController.run() * 2;

    var t = config.turret;
    double power = Math.clamp(Math.abs(pidOutput), t.min_turn_power, t.max_turn_power);
    power = Math.copySign(power, pidOutput);

    leftFront.setPower(-power);
    leftBack.setPower(-power);
    rightFront.setPower(power);
    rightBack.setPower(power);

    double distance = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
    double dynamicToleranceDegrees = calculateDynamicTolerance(distance);
    double dynamicToleranceRadians = Math.toRadians(dynamicToleranceDegrees);

    telemetry.addData("Turn Target (Deg)", targetHeadingDegrees);
    telemetry.addData("Turn Current (Deg)", Math.toDegrees(currentHeadingRad));
    telemetry.addData("Turn Error (Rad)", signedError);
    telemetry.addData("Turn Error (Deg)", Math.toDegrees(errorAbs));
    telemetry.addData("Turn Tolerance (Deg)", dynamicToleranceDegrees);
    telemetry.addData("PID Output", pidOutput);

    if (errorAbs < dynamicToleranceRadians) {
      stopDrive();
      headingController.reset();
      return false;
    }

    return true;
  }

  public void stopDrive() {
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
  }

  /** Drives all four wheels to turn the robot in place, for turning-power calibration. */
  public void driveTurn(double power) {
    leftFront.setPower(-power);
    rightFront.setPower(power);
    leftBack.setPower(-power);
    rightBack.setPower(power);
  }

  /** Drives all four wheels forward, for forward-power calibration. */
  public void driveForward(double power) {
    leftFront.setPower(power);
    rightFront.setPower(power);
    leftBack.setPower(power);
    rightBack.setPower(power);
  }

  /** Drives all four wheels to strafe the robot sideways, for strafe-power calibration. */
  public void driveStrafe(double power) {
    leftFront.setPower(-power);
    rightFront.setPower(power);
    leftBack.setPower(power);
    rightBack.setPower(-power);
  }

  /** Drives the turret servo directly, for turret-power calibration. */
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
    double robotWorldHeading = currentPose.getHeading();
    double turretWorldAngle = getCurrentTurnAngle();
    double targetWorldAngle = targetTurnAngle;

    double relativeTurretAngle = (turretWorldAngle - robotWorldHeading + 360) % 360;
    double relativeTargetAngle = (targetWorldAngle - robotWorldHeading + 360) % 360;

    boolean turnLeft = shouldTurnLeft(relativeTurretAngle, relativeTargetAngle);
    return computeSignedError(relativeTurretAngle, relativeTargetAngle, turnLeft);
  }

  public boolean isAimed(Pose currentPose) {
    double error = getAimError(currentPose);
    double distance = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
    double dynamicTolerance = calculateDynamicTolerance(distance);
    return Math.abs(error) < dynamicTolerance;
  }

  public void setAimMode(AimMode mode) {
    this.mode = mode;
  }

  public AimMode getAimMode() {
    return mode;
  }

  public void setHoldAngle(double angle) {
    this.holdAngle = angle;
  }

  public double getHoldAngle() {
    return holdAngle;
  }

  public void periodic() {
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
          double deltaX = goalX - pose.getX();
          double deltaY = goalY - pose.getY();
          this.targetTurnAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
          updateTurret(pose);
        }
        break;
    }
  }
}
