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
import org.firstinspires.ftc.teamcode.utilities.ConfigLoader;

@Configurable
public class Turret {
    public static double MAX_TURN_POWER = ConfigLoader.getDouble("turret.max_turn_power");
    public static double MIN_TURN_POWER = ConfigLoader.getDouble("turret.min_turn_power");
    public static double MAX_POWER_OUTPUT = ConfigLoader.getDouble("turret.max_power_output");
    public static double FEED_FORWARD = ConfigLoader.getDouble("turret.feed_forward");

    public static double CACHING_MOTOR_TOLERANCE = ConfigLoader.getDouble("caching.drivetrain_tolerance");

    public static double PID_P = ConfigLoader.getDouble("turret.pidf.p");
    public static double PID_I = ConfigLoader.getDouble("turret.pidf.i");
    public static double PID_D = ConfigLoader.getDouble("turret.pidf.d");
    public static double PID_F = ConfigLoader.getDouble("turret.pidf.f");

    public static double TURN_OFFSET = ConfigLoader.getDouble("turret.turn.offset_const");
    public static double TURN_LIMIT = ConfigLoader.getDouble("turret.turn.limit_const");

    public static double MAX_DRIFT = ConfigLoader.getDouble("turret.tolerance.max_drift");
    public static double NEAR_CUTOFF = ConfigLoader.getDouble("turret.tolerance.near_cutoff");
    public static double NEAR_VAL = ConfigLoader.getDouble("turret.tolerance.near_val");
    public static double MIN_DEG = ConfigLoader.getDouble("turret.tolerance.min_deg");
    public static double MAX_DEG = ConfigLoader.getDouble("turret.tolerance.max_deg");

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

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        this.telemetry = telemetry;

        turnServo = new CachingCRServo(hardwareMap.get(CRServo.class, "turn"));
        turnIMU = hardwareMap.get(IMU.class, "turnImu");

        DcMotorEx lf  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx lb  = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rf  = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rb  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront  = (lf instanceof CachingDcMotorEx) ? (CachingDcMotorEx) lf : new CachingDcMotorEx(lf);
        leftBack   = (lb instanceof CachingDcMotorEx) ? (CachingDcMotorEx) lb : new CachingDcMotorEx(lb);
        rightFront = (rf instanceof CachingDcMotorEx) ? (CachingDcMotorEx) rf : new CachingDcMotorEx(rf);
        rightBack  = (rb instanceof CachingDcMotorEx) ? (CachingDcMotorEx) rb : new CachingDcMotorEx(rb);

        leftFront.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        leftBack.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        rightFront.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        rightBack.setCachingTolerance(CACHING_MOTOR_TOLERANCE);

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.valueOf(ConfigLoader.getString("turret.orientation.logo"));
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.valueOf(ConfigLoader.getString("turret.orientation.usb"));
        turnIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        turnIMU.resetYaw();

        PIDFCoefficients headingPIDFCoefficients = Constants.followerConstants.getCoefficientsHeadingPIDF();
        headingController = new PIDFController(headingPIDFCoefficients);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
        pidfController = new PIDFController(pidfCoefficients);
    }

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        turnServo = new CachingCRServo(hardwareMap.get(CRServo.class, "turn"));
        turnIMU = hardwareMap.get(IMU.class, "turnImu");

        DcMotorEx lf  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx lb  = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rf  = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rb  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront  = (lf instanceof CachingDcMotorEx) ? (CachingDcMotorEx) lf : new CachingDcMotorEx(lf);
        leftBack   = (lb instanceof CachingDcMotorEx) ? (CachingDcMotorEx) lb : new CachingDcMotorEx(lb);
        rightFront = (rf instanceof CachingDcMotorEx) ? (CachingDcMotorEx) rf : new CachingDcMotorEx(rf);
        rightBack  = (rb instanceof CachingDcMotorEx) ? (CachingDcMotorEx) rb : new CachingDcMotorEx(rb);

        leftFront.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        leftBack.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        rightFront.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        rightBack.setCachingTolerance(CACHING_MOTOR_TOLERANCE);

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.valueOf(ConfigLoader.getString("turret.orientation.logo"));
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.valueOf(ConfigLoader.getString("turret.orientation.usb"));
        turnIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        turnIMU.resetYaw();

        PIDFCoefficients headingPIDFCoefficients = Constants.followerConstants.getCoefficientsHeadingPIDF();
        headingController = new PIDFController(headingPIDFCoefficients);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
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

    public static Pose AlignPose(double X, double Y, double GOALX, double GOALY) {
        double deltaX = GOALX - X;
        double deltaY = GOALY - Y;
        double angleRadians = Math.atan2(deltaY, deltaX);
        return new Pose(X, Y, angleRadians);
    }

    public double calculateDynamicTolerance(double distanceInches) {
        if (distanceInches < NEAR_CUTOFF) return NEAR_VAL;
        double ratio = Math.clamp((MAX_DRIFT / 2.0) / distanceInches, -1.0, 1.0);
        double thetaRadians = Math.asin(ratio);
        return Math.clamp(Math.toDegrees(thetaRadians), MIN_DEG, MAX_DEG);
    }

    public boolean shouldTurnLeft(double current, double target) {
        double cR = (current + TURN_OFFSET) % 360;
        double tR = (target + TURN_OFFSET) % 360;

        boolean lBad = cR <= tR ? (cR <= TURN_LIMIT && tR >= 0) : (cR <= TURN_LIMIT || tR >= 0);
        boolean rBad = tR <= cR ? (tR <= TURN_LIMIT && cR >= 0) : (tR <= TURN_LIMIT || cR >= 0);

        double ld = (target - current + 360) % 360;
        double rd = (current - target + 360) % 360;

        return lBad == rBad ? (ld <= rd) : !lBad;
    }

    public void updateTurret(Pose currentPose) {
        double robotWorldHeading = currentPose.getHeading();
        double turretWorldAngle = turnIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Math.toDegrees(initialHeadingOffset);
        double targetWorldAngle = targetTurnAngle;

        double relativeTurretAngle = (turretWorldAngle - robotWorldHeading + 360) % 360;
        double relativeTargetAngle = (targetWorldAngle - robotWorldHeading + 360) % 360;

        boolean turnLeft = shouldTurnLeft(relativeTurretAngle, relativeTargetAngle);

        double error;
        if (turnLeft) {
            error = (relativeTargetAngle - relativeTurretAngle + 360) % 360;
            if (error > 180) error -= 360;
        } else {
            error = -((relativeTurretAngle - relativeTargetAngle + 360) % 360);
            if (error < -180) error += 360;
        }

        if (Math.abs(error) < calculateDynamicTolerance(Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY()))) {
            turnServo.setPower(0);
            isTurnDone = true;
            pidfController.reset();
        } else {
            isTurnDone = false;

            pidfController.setTargetPosition(relativeTurretAngle + error);
            pidfController.updatePosition(relativeTurretAngle);
            pidfController.updateFeedForwardInput(1);
            double power = pidfController.run();

            double absPower = (power == 0) ? 0 : Math.clamp(Math.abs(power), FEED_FORWARD, MAX_POWER_OUTPUT);

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

        double power = Math.clamp(Math.abs(pidOutput), MIN_TURN_POWER, MAX_TURN_POWER);
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

    public void tuneMinTurnPower(double min) {
        leftFront.setPower(-min);
        rightFront.setPower(min);
        leftBack.setPower(-min);
        rightBack.setPower(min);
    }

    public void tunemin(double min) {
        leftFront.setPower(min);
        rightFront.setPower(min);
        leftBack.setPower(min);
        rightBack.setPower(min);
    }

    public void tuneminside(double min) {
        leftFront.setPower(-min);
        rightFront.setPower(min);
        leftBack.setPower(min);
        rightBack.setPower(-min);
    }

    public void tuneMinTurretPower(double min) {
        turnServo.setPower(min);
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
}
