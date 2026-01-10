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
    private static final int MAX_RPM = 1500;
    private static final double INTAKE_TRANSFER_POWER = 1.0;
    private static final double HOOD_LONG_POSITION = 0.35;
    private static final double TURN_TOLERANCE_DEGREES = 2;
    private static final double MAX_TURN_ANGLE = 100.0; // Maximum positive rotation
    private static final double MIN_TURN_ANGLE = -100.0; // Maximum negative rotation

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.009, 0.0003, 0.003, 0.1); // (0.008, 0.0005, 0.003, 0.007);
    PIDFController pidfController = new PIDFController(pidfCoefficients);

    public static double robotTurnPinPoint_Kp = 0.025;

    // PIDF Constants
    public static double TURN_P = 0.01;
    public static double TURN_I = 0.0005;
    public static double TURN_D = 0.003;
    public static double TURN_F = 0.07;

    // Safety limits
    private static final double MAX_INTEGRAL = 5.0; // Prevent integral windup
    private static final double MAX_POWER_OUTPUT = 1;
    private static final double DERIVATIVE_FILTER = 0.7; // Low-pass filter for derivative

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

    // PID State
    private double lastError = 0;
    private double integralSum = 0;
    private double lastDerivative = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

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
        pidfController.reset();
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
        // Clamp to physical limits
        this.targetTurnAngle = Math.max(MIN_TURN_ANGLE, Math.min(MAX_TURN_ANGLE, degrees));
        pidfController.setTargetPosition(this.targetTurnAngle);
    }

    public void turnTurret() {
        double turn_degrees = targetTurnAngle - Math.toDegrees(initialHeadingOffset);
        telemetry.addData("Target turn in degrees ", turn_degrees);
        double maxPower = 0.5, minPower = 0.13;
        double Kp = 0.006;
        double currentAngle = turnIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (turn_degrees > 115) {
            turn_degrees = 115;
        } else if (turn_degrees < -115) {
            turn_degrees = -115;
        }
        double error = turn_degrees - currentAngle;
        turnServo.setDirection(CRServo.Direction.FORWARD);
        while (Math.abs(error) > 2) {
            isTurnDone = false;
            currentAngle = turnIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turn_degrees - currentAngle;
            double power = error * Kp;
            if (power > maxPower) {
                power = maxPower;
            } else if (power < minPower && power > 0) {
                power = minPower;
            } else if (power > -minPower && power < 0) {
                power = -minPower;
            } else if (power < -maxPower) {
                power = -maxPower;
            }
            telemetry.addData("Current Power", power);
            telemetry.addData("Error: ", error);
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
            turnServo.setPower(power);
        }
        turnServo.setPower(0);
        isTurnDone = true;
    }

    public void updateTurret() {
        YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.RADIANS);
        double currentDegrees = Math.toDegrees(currentYaw + initialHeadingOffset);

        // Calculate distances for both directions
        double ld = (targetTurnAngle - currentDegrees + 360) % 360;
        double rd = (currentDegrees - targetTurnAngle + 360) % 360;

        boolean turnLeft = shouldTurnLeft(currentDegrees, targetTurnAngle);
        double error = turnLeft ? ld : rd;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (Math.abs(error) < TURN_TOLERANCE_DEGREES) {
            turnServo.setPower(0);
            isTurnDone = true;
            integralSum = 0;
            lastError = 0;
        } else {
            isTurnDone = false;

            // PID Calculations with safety limits
            integralSum += error * dt;
            // Clamp integral to prevent windup
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));

            double derivative = (error - lastError) / dt;
            // Apply low-pass filter to derivative to reduce noise spikes
            derivative = DERIVATIVE_FILTER * derivative + (1 - DERIVATIVE_FILTER) * lastDerivative;
            lastDerivative = derivative;
            lastError = error;

            double pidOutput = (error * TURN_P) + (integralSum * TURN_I) + (derivative * TURN_D);
            double feedforward = TURN_F;

            // SAFETY: Cap total power output to prevent hardware damage
            double power = Math.clamp(pidOutput + feedforward, 0, MAX_POWER_OUTPUT);

            // Apply minimum power threshold to prevent weak jittery movements
            if (power < 0.05) {
                power = 0;
            }

            if (turnLeft) {
                turnServo.setDirection(CRServo.Direction.REVERSE);
            } else {
                turnServo.setDirection(CRServo.Direction.FORWARD);
            }
            turnServo.setPower(power);
        }

        telemetry.addData("Turret Yaw", currentDegrees);
        telemetry.addData("Turret Target", targetTurnAngle);
        telemetry.addData("Turret Error", error);
        telemetry.addData("Turret Power", turnServo.getPower());
        telemetry.addData("Turret Done", isTurnDone);
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
