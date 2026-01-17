package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

@Configurable
public class IntakeLauncher {

    private final PIDFCoefficients headingPIDFCoefficients = Constants.followerConstants.getCoefficientsHeadingPIDF();
    private final PIDFController headingController = new PIDFController(headingPIDFCoefficients);
    private static final double MAX_TURN_POWER = 0.5; // Safety clamp
    private static final double MIN_TURN_POWER = 0.12;

    // Constants
    private static final int MAX_RPM = 1450;
    private static final double INTAKE_TRANSFER_POWER = 1.0;
    public static double HOOD_LONG_POSITION = 0.4;

    public static double minTransferThreashhold = .95;

    // Shooting Mode Configuration
    public static boolean AUTO_SHOOT_MODE = true; // true = right trigger does full sequence, false = two-button mode (X then trigger)

    // PIDF Constants (original working values)
//    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.01, 0.0005, 0.003, 0);
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.01835, 0, 0.00135, 0);

    double f = 0.08;
    // PIDF f doesn't work, set to 0, and adjust f value above
    PIDFController pidfController = new PIDFController(pidfCoefficients);

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


    public IntakeLauncher(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
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

        follower.getDrivetrain().updateConstants();

        // Drive Motors
        // Configure Turn IMU
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        turnIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        turnIMU.resetYaw();
        setHoodPosition(0);
    }

    // WARNING: USE THIS ONLY IF YOU ARENT USING DRIVETRAIN
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

    public void runShooterRaw(double power) {
        // Set raw power, bypassing the REV Hub's internal PID
        shooter.setPower(power);
        shooter2.setPower(power);
    }

    public double getShooterVelocity() {
        // Returns velocity in Ticks Per Second
        return shooter.getVelocity();
    }

    public double getShooterPower() {
        return shooter.getPower();
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
        double targetVel = MAX_RPM * power;
        shooter.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);
    }

    public double calculateDynamicTolerance(double distanceInches) {
        final double MAX_DRIFT_ALLOWED = 6.0;

        if (distanceInches < 1.0) return 45.0;

        double thetaRadians = Math.asin((MAX_DRIFT_ALLOWED / 2.0) / distanceInches);

        return Math.clamp(Math.toDegrees(thetaRadians), 0.5, 2.0);
    }

    public void takeShot(double launchPower){
        if (!isShooting) return;
        double targetVel = MAX_RPM * launchPower;
        shooter.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);
        if (shooter.getVelocity() > (targetVel * minTransferThreashhold)) {
            intakeMid.setPower(INTAKE_TRANSFER_POWER);
            intakeFront.setPower(INTAKE_TRANSFER_POWER);
        } else {
            intakeMid.setPower(0);
            intakeFront.setPower(0);
        }
    }

    public void updateShootingLogic(double launchPower, Pose currentPose) {
        if (!isShooting) return;

        double targetVel = MAX_RPM * launchPower;
        shooter.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);

        telemetry.addData("current v", Math.abs(shooter.getVelocity()));
        telemetry.addData("target v", Math.abs(targetVel) * minTransferThreashhold);

        // Calculate turret alignment error
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

        // Calculate dynamic tolerance based on distance to goal
        double distance = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
        double dynamicTolerance = calculateDynamicTolerance(distance);

        // Feed ball when shooter is ready (98% of target velocity) AND turret is within dynamic tolerance
        boolean velocityReady = Math.abs(shooter.getVelocity()) > Math.abs(targetVel) * minTransferThreashhold &&
                                Math.abs(shooter.getVelocity()) < Math.abs(targetVel) * 1.05;
        boolean turretAligned = Math.abs(error) < dynamicTolerance;

        telemetry.addData("Turret Error (deg)", error);
        telemetry.addData("Dynamic Tolerance (deg)", dynamicTolerance);
        telemetry.addData("Turret Aligned", turretAligned);

        if (velocityReady) {
            intakeMid.setPower(INTAKE_TRANSFER_POWER);
            intakeFront.setPower(INTAKE_TRANSFER_POWER);
        } else {
            intakeMid.setPower(0);
            intakeFront.setPower(0);
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
            launchPower = 0.59;
            waitTime = 2000;
        } else {
            launchPower = 0.62 + ((distance - 33) / 300.0);
            waitTime = distance * 68;
        }

        return new LaunchParameters(launchPower, waitTime, Math.toDegrees(angleRadians));
    }

    public static Pose AlignPose(double X, double Y, double GOALX, double GOALY) {
        double deltaX = GOALX - X;
        double deltaY = GOALY - Y;
        double angleRadians = Math.atan2(deltaY, deltaX);
        return new Pose(X, Y, angleRadians);
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

    public void updateTurret(Pose currentPose) {
        double robotWorldHeading = currentPose.getHeading();
        // Get turret's current world angle
        double turretWorldAngle = turnIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Math.toDegrees(initialHeadingOffset);

        // FIX: targetTurnAngle is now already in degrees
        double targetWorldAngle = targetTurnAngle+2.5;

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

    public void setShooterPIDFCoefficients(){
        com.qualcomm.robotcore.hardware.PIDFCoefficients shooterPIDFCoefficients= new  com.qualcomm.robotcore.hardware.PIDFCoefficients(150, 0, 0, 22.8);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDFCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDFCoefficients);
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

    /**
     * Updates the robot turn using PedroPathing's tuned PIDF constants.
     * Call this in your loop.
     *
     * @param currentPose The current Pose from pinpoint/follower.
     * @param targetHeadingDegrees The target angle in DEGREES.
     * @return true if the turn is complete (within tolerance).
     */
    public boolean updateTurn(Pose currentPose, double targetHeadingDegrees) {
        // 1. Convert everything to Radians (PedroPathing Constants are tuned for Radians)
        double currentHeadingRad = currentPose.getHeading();
        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);

        // 2. Calculate Error using PedroPathing MathFunctions
        // getSmallestAngleDifference returns positive magnitude
        double errorAbs = MathFunctions.getSmallestAngleDifference(currentHeadingRad, targetHeadingRad);

        // getTurnDirection returns 1 (CCW/Left) or -1 (CW/Right)
        double direction = MathFunctions.getTurnDirection(currentHeadingRad, targetHeadingRad);

        // Combined signed error in Radians
        double signedError = errorAbs * direction;

        // 3. Update the PedroPathing Controller
        headingController.updateFeedForwardInput(direction);
        headingController.updateError(signedError);

        // 4. Run PIDF
        // Note: We leave FeedForward input as 0 for point turns, relying on P and D.
        double pidOutput = headingController.run() * 2;

        // 5. Clamp and Apply Power
        // If pidOutput is positive (Left Turn), we want Left Motors Reverse, Right Motors Forward
        double power = Math.clamp(Math.abs(pidOutput), MIN_TURN_POWER , MAX_TURN_POWER);
        power = Math.copySign(power, pidOutput);

        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        // Calculate dynamic tolerance based on distance to goal
        double distance = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
        double dynamicToleranceDegrees = calculateDynamicTolerance(distance);
        double dynamicToleranceRadians = Math.toRadians(dynamicToleranceDegrees);

        // Telemetry for debugging
        telemetry.addData("Turn Target (Deg)", targetHeadingDegrees);
        telemetry.addData("Turn Current (Deg)", Math.toDegrees(currentHeadingRad));
        telemetry.addData("Turn Error (Rad)", signedError);
        telemetry.addData("Turn Error (Deg)", Math.toDegrees(errorAbs));
        telemetry.addData("Turn Tolerance (Deg)", dynamicToleranceDegrees);
        telemetry.addData("PID Output", pidOutput);

        // 6. Check for Completion using dynamic tolerance
        if (errorAbs < dynamicToleranceRadians) {
            stopDrive();
            headingController.reset(); // Reset for next time
            return false;
        }

        return true;
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

    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
