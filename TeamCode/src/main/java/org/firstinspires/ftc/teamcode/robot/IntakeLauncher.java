package org.firstinspires.ftc.teamcode.robot;
import static java.lang.Thread.sleep;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.pedropathing.geometry.Pose;

public class IntakeLauncher {

    // Constants
    private static final double TRAIN_RPM_PERCENT = 1.0;
    private static final int MAX_RPM = 1450;
    private static final double INTAKE_TRANSFER_POWER = 1.0;
    private static final double HOOD_LONG_POSITION = 0.4;
    private static final double TURN_TOLERANCE_DEGREES = 2;

    // PIDF Constants (original working values)
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.01, 0.0005, 0.003, 0);
    double f = 0.13;
    // PIDF f doesn't work, set to 0, and adjust f value above
    PIDFController pidfController = new PIDFController(pidfCoefficients);

    public static double robotTurnPinPoint_Kp = 0.025;
    private static final double MAX_POWER_OUTPUT = 0.5;

    // Hardware
    private final DcMotorEx intakeFront,intakeMid;
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private final DcMotorEx shooter, shooter2;
    GoBildaPinpointDriver pinpoint;
    private final Servo hood;
    private final CRServo turnServo;
    private final IMU turnIMU;
    private final Telemetry telemetry;

    // State
    private double goalX;
    private double goalY;
    private double targetTurnAngle = 90;
    private double initialHeadingOffset = 0;
    private boolean isShooting = false;
    private boolean isTurnDone = false;
    private final ElapsedTime shootingTimer = new ElapsedTime();


    public IntakeLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Initialize Hardware
        turnIMU = hardwareMap.get(IMU.class, "turnImu");
        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeMid = hardwareMap.get(DcMotorEx.class, "intakeMid");
        hood = hardwareMap.get(Servo.class, "hood");
        turnServo = hardwareMap.get(CRServo.class, "turn");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //configure pinpoint
        pinpoint.setOffsets(1.5, -4.75, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Configure Motors
        intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMid.setDirection(DcMotorSimple.Direction.REVERSE);

        // shooter motor
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);


        // shooter2 motor
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Drive Motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        // Configure Turn IMU
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        turnIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        turnIMU.resetYaw();
        setHoodPosition(0);
    }

    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public void setInitialHeading(double heading) {
        this.initialHeadingOffset = heading;
    }

//    public void setFollower(Follower follower){
//        this.follower = follower;
//    }
    public void runIntake(double intakePower, double transferPower) {
        intakeFront.setPower(intakePower);
        intakeMid.setPower(transferPower);
    }

    public void stopIntake() {
        intakeFront.setPower(0);
        intakeMid.setPower(0);
    }

    public void stopLauncher() {
        shooter.setPower(0);
        shooter2.setPower(0);
        stopIntake();
    }

    public void powerOnLauncher(double power) {
        double targetVel = MAX_RPM * power * TRAIN_RPM_PERCENT;
        shooter.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);
    }

    public void updateShootingLogic(double launchPower) {
        if (!isShooting) return;

        double targetVel = MAX_RPM * launchPower;
        shooter.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);

        telemetry.addData("current v", Math.abs(shooter.getVelocity()));
        telemetry.addData("target v", Math.abs(targetVel) * 0.97);

        // Feed ball when shooter is ready (93% of target velocity)
        if (Math.abs(shooter.getVelocity()) > Math.abs(targetVel) * 0.97) {
            intakeMid.setPower(INTAKE_TRANSFER_POWER);
            intakeFront.setPower(INTAKE_TRANSFER_POWER);
        } else {
            intakeMid.setPower(0);
            intakeFront.setPower(0.1);
        }
    }

    public LaunchParameters calculateLaunchParameters(Pose currentPose) {
        double deltaX = goalX - currentPose.getX();
        double deltaY = goalY - currentPose.getY();
        double distance = Math.hypot(deltaX, deltaY);
        double angleRadians = Math.atan2(deltaY, deltaX);

        double launchPower;
        double waitTime;

        if (distance <= 33) {
            launchPower = 0.57;
            waitTime = 2000;
        } else {
            launchPower = 0.57 + ((distance - 33) / 320.0);
            waitTime = distance * 68;
        }

        return new LaunchParameters(launchPower, waitTime, Math.toDegrees(angleRadians));
    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public void setHoodLongShotPosition() {
        hood.setPosition(HOOD_LONG_POSITION);
    }

    public void setTargetTurnAngle(double degrees) {
        this.targetTurnAngle = degrees;
    }

    public void updateTurret(double robotWorldHeading) {
        // Get turret's current world angle
        double turretWorldAngle = turnIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Math.toDegrees(initialHeadingOffset);

        // FIX: targetTurnAngle is now already in degrees
        double targetWorldAngle = targetTurnAngle;

        // Convert to robot-relative coordinates
        double relativeTurretAngle = (turretWorldAngle - robotWorldHeading + 360) % 360;
        double relativeTargetAngle = (targetWorldAngle - robotWorldHeading + 360) % 360;

        // Now shouldTurnLeft works correctly because angles are robot-relative
        boolean turnLeft = shouldTurnLeft(relativeTurretAngle, relativeTargetAngle);

        // Calculate error in robot-relative space
        double error;
        if (turnLeft) {
            error = (relativeTargetAngle - relativeTurretAngle + 360) % 360;
            if (error > 180) error -= 360;
        } else {
            error = -((relativeTurretAngle - relativeTargetAngle + 360) % 360);
            if (error < -180) error += 360;
        }

        if (Math.abs(error) < TURN_TOLERANCE_DEGREES) {
            turnServo.setPower(0);
            isTurnDone = true;
            pidfController.reset();
        } else {
            isTurnDone = false;

            pidfController.setTargetPosition(relativeTurretAngle + error);
            pidfController.updatePosition(relativeTurretAngle);
            pidfController.updateFeedForwardInput(1);
            double power = pidfController.run();

            double absPower = Math.abs(power);
            if (power != 0) absPower = Math.clamp(absPower, f, MAX_POWER_OUTPUT);

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

    public boolean shouldTurnLeft(double current, double target) {
        double cR = (current + 260) % 360;
        double tR = (target + 260) % 360;
        final double F_END = 160;

        boolean lBad = cR <= tR ? (cR <= F_END && tR >= 0) : (cR <= F_END || tR >= 0);
        boolean rBad = tR <= cR ? (tR <= F_END && cR >= 0) : (tR <= F_END || cR >= 0);

        double ld = (target - current + 360) % 360;
        double rd = (current - target + 360) % 360;

        return lBad == rBad ? (ld <= rd) : !lBad;
    }

    public boolean isTurnDone() {
        return isTurnDone;
    }

    public void startShooting() {
        isShooting = true;
        shootingTimer.reset();
    }

    public void stopShooting() {
        isShooting = false;
        stopLauncher();
    }

    public boolean isShooting() {
        return isShooting;
    }

    public double getShootingDuration() {
        return shootingTimer.milliseconds();
    }

    public double getCurrentTurnAngle() {
        YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
        return Math.toDegrees(orientation.getYaw(AngleUnit.RADIANS) + initialHeadingOffset);
    }

    public void turnRobot() {
        double min_power = 0.2, max_power = 0.4;
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        double currentAngle = pose2D.getHeading(AngleUnit.DEGREES);
        double error = targetTurnAngle - currentAngle;

        telemetry.addData("Start: Robot Current Angle", currentAngle);
        telemetry.addData("Start: Robot Turn Target", targetTurnAngle);
        // Handle angle wrapping (e.g., turning from 170 to -170 degrees)
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        double power = error * robotTurnPinPoint_Kp;
        // Clamp power to a reasonable range (e.g., -0.5 to 0.5) to prevent twitching
        power = Math.max(-max_power, Math.min(max_power, power));
        telemetry.addData("Start: Robot Turn Power", power);
        telemetry.update();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        // Apply power (opposite for left/right)
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
        while (Math.abs(error) > 1) { // Loop until very close
            isTurnDone = false;
            pinpoint.update();
            pose2D = pinpoint.getPosition();
            currentAngle = pose2D.getHeading(AngleUnit.DEGREES);
//            currentAngle = follower.getHeading();
            error = targetTurnAngle - currentAngle;
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            power = error * robotTurnPinPoint_Kp;
            telemetry.addData("Robot Current Angle", currentAngle);
            telemetry.addData("Robot Turn Target", targetTurnAngle);
            telemetry.addData("Robot Turn power", power);
            telemetry.update();
            if (power > 0 && power < min_power) {
                power = min_power;
            } else  if (power < 0 && power > -min_power) {
                power = -min_power;
            } else {
                power = Math.max(-max_power, Math.min(max_power, power));
            }
            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftBack.setPower(-power);
            rightBack.setPower(power);
        }
        isTurnDone = true;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
