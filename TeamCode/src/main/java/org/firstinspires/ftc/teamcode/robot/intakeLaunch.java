package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class intakeLaunch {

    private DcMotorEx intakeF, intakeM, shooter;
    private IMU turnIMU;
    private Servo hood;
    private CRServo turn;
    private final Telemetry telemetry;
    public static double TRAIN_RPM_PERCENT = 1;
    public static double LAUNCH_VELOCITY = .7;
    public static int MAX_RPM = 1500;
    public static double INTAkE_TRANSFER_POWER = 1;

    public static double degrees = 90;

    double long_position = .35;
    private DistanceSensor intakeSensor;

    private static final int MAX_BALLS = 3;
    private static final double DISTANCE_THRESHOLD = 8.5;
    private static final double STUCK_TIME = 500;

    private int ballCount = 0;
    private double[] ballDistances = new double[MAX_BALLS];
    private boolean ballDetectedPreviously = false;
    private double firstTimeBallDetected = -1;
    private boolean intakeFull = false;

    public static double p = 0.001, f = 0.06;

    public static double currentTurnAngle;

    public static boolean done = false;

    public static double initial;

    public static double goalX = 129;
    public static double goalY = 131;

    public static double tolerance = 2;

    public intakeLaunch(HardwareMap hardwareMap, Telemetry tel) {
        telemetry = tel;
        turnIMU = hardwareMap.get(IMU.class, "turnImu");
        //Intake Motors
        intakeF = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeM = hardwareMap.get(DcMotorEx.class, "intakeMid");
        hood = hardwareMap.get(Servo.class, "hood");
        turn = hardwareMap.get(CRServo.class, "turn");
//        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeD");
        intakeF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        intakeF.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeM.setDirection(DcMotorSimple.Direction.REVERSE);

        //Shooter Motor
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // TODO: reverse motor directions if needed
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        setHoodPosition(0);
//        setTurnPosition(0.5);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        turnIMU.initialize(new IMU.Parameters(orientationOnRobot));
        turnIMU.resetYaw();
    }

    public void runIntake(double intakePower, double transferPower) {
        telemetry.addData("Setting intake power to", intakePower);
        telemetry.addData("Setting transfer power to", transferPower);
        intakeF.setPower(intakePower);
        intakeM.setPower(transferPower);
        telemetry.addData("intake Front velocity ", intakeF.getVelocity());
    }

    public void stopIntake() {
        telemetry.addData("Shutting down: ", "intake");
        intakeM.setPower(0);
        intakeF.setPower(0);
    }

    public void takeShot(double launchPower, double waitTime) {
        telemetry.addData("Launching balls with power ", launchPower);
        ElapsedTime runtime = new ElapsedTime();
        double targetVel = MAX_RPM * launchPower;
        while (runtime.milliseconds() < waitTime) {
            shooter.setVelocity(targetVel);
            // start feeding the ball as soon as launcher velocity reaches 90% of the target to reduce the transfer time
            if (abs(shooter.getVelocity()) > MAX_RPM * (0.9 * launchPower)) {
                telemetry.addData("start the intake during shot ", shooter.getVelocity());
                intakeM.setPower(INTAkE_TRANSFER_POWER);
                intakeF.setPower(INTAkE_TRANSFER_POWER);
            } else {
                intakeM.setPower(0);
                intakeF.setPower(0.1);
            }
        }
    }

    public void powerOnLauncher(double launchPower) {
        telemetry.addData("Turning on Launcher with power ", launchPower);
        double targetVel = MAX_RPM * launchPower * TRAIN_RPM_PERCENT;
        shooter.setVelocity(targetVel);
    }

    public void stopLauncher() {
        telemetry.addData("Shutting down: ", "Launcher");
        shooter.setPower(0);
        intakeM.setPower(0);
        intakeF.setPower(0);
    }

    public void waitForMSeconds(int ms) {
        try {
            sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public double getPoseDistance(Pose CurrentPose) {
        telemetry.addData("calculating distance for ", CurrentPose.getX() + ", " + CurrentPose.getY());
        double deltaX = goalX - CurrentPose.getX();
        double deltaY = goalY - CurrentPose.getY();
        // Uses Math.sqrt() and Math.pow() or simply multiplication
        double distanceToGoal = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
//        double distanceToGoal = Math.hypot(goalTargetPose.getX() - CurrentPose.getX(), goalTargetPose.getY() - CurrentPose.getY());
        telemetry.addData("Distance is ", distanceToGoal);
        return distanceToGoal;
    }

    public double getAngeleToGoal(Pose CurrentPose) {
        telemetry.addData("calculating angle for ", CurrentPose.getX() + ", " + CurrentPose.getY());
        double angleRadians = Math.atan2(goalY - CurrentPose.getY(), goalX - CurrentPose.getX());
        telemetry.addData("Angle is ", angleRadians);
//        return Math.toDegrees(angleRadians);
        return angleRadians;
    }

    public LaunchParameters calculateLaunchParameters(Pose CurrentPose) {
        telemetry.addData("calculating launch parameters for ", CurrentPose.getX() + ", " + CurrentPose.getY());
        LaunchParameters LP;
        double distanceToGoal = getPoseDistance(CurrentPose);
        double angleToGoal = getAngeleToGoal(CurrentPose);
        double launchPower = 0;
        double waitTime = 0;
        if (distanceToGoal <= 33) {
            launchPower = 0.6;
            waitTime = 2400;
        } else {
            launchPower = 0.61 + ((distanceToGoal - 33) / 300);
            waitTime = distanceToGoal * 65;
        }

        telemetry.addData("Launch parameters are Power:", launchPower + ", Angle:" + Math.toDegrees(angleToGoal) + ", Distance:" + distanceToGoal + ", wait Time is:" + waitTime);
        telemetry.addData("launch parameters are ", CurrentPose.getX() + ", " + CurrentPose.getY());

//        try {
//            sleep(5000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }

        LP = new LaunchParameters(launchPower, waitTime, Math.toDegrees(angleToGoal));
        return LP;

    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public void setHoodLongShotPosition() {
        hood.setPosition(long_position);
    }

    public boolean shouldTurnLeft(double c, double t) {
        // 1. Rotate coordinates by +260 degrees (or -100) so the forbidden zone [100, 260]
        //    becomes the continuous range [0, 160] in the rotated system.
        double cR = (c + 260) % 360, tR = (t + 260) % 360;

        // 2. The forbidden range is F = [0, 160]
        final double F_END = 160;

        // 3. Check if Left (CCW) path intersects F
        // Path: cR to tR CCW.
        boolean lBad = cR <= tR ?
                (cR <= F_END && tR >= 0) : // Non-wrapping: starts before 160 AND ends after 0
                (cR <= F_END || tR >= 0);  // Wrapping: starts before 160 OR ends after 0

        // 4. Check if Right (CW) path intersects F (Path: tR to cR CCW)
        boolean rBad = tR <= cR ?
                (tR <= F_END && cR >= 0) :
                (tR <= F_END || cR >= 0);

        // 5. Shortest path calculation (not rotated)
        double ld = (t - c + 360) % 360, rd = (c - t + 360) % 360;

        // 6. If both paths are bad/safe, take shortest (ld <= rd). Otherwise, take the safe path (!lBad).
        return lBad == rBad ? (ld <= rd) : !lBad;
    }

    public void setTurnPosition() {
        YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = turnIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
        currentTurnAngle = orientation.getYaw(AngleUnit.RADIANS);
        currentTurnAngle += initial;
        currentTurnAngle = Math.toDegrees(currentTurnAngle);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", currentTurnAngle);
        if (shouldTurnLeft(currentTurnAngle, degrees)) {
            turn.setDirection(CRServo.Direction.REVERSE);
        } else {
            turn.setDirection(CRServo.Direction.FORWARD);
        }
//        while (((currentTurnAngle * direction) - degrees)  > 2 ) {
//            turn.setPower(1);
//            currentTurnAngle = orientation.getYaw(AngleUnit.DEGREES);
//        }
        turn.setPower(Math.abs(degrees  - currentTurnAngle)*p + f);
        if (Math.abs(degrees  - currentTurnAngle ) < tolerance) {
            done = true;
            turn.setPower(0);
        }
        else {
            done = false;
        }
//        time = (int) (Math.abs(degrees) * 4.2);
//        turn.setPower(1);
//        try {
//            sleep(time);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        turn.setPower(0);
    }

    public void turnLauncher(double degrees) {
//        YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
//        AngularVelocity angularVelocity = turnIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
//        currentTurnAngle = orientation.getYaw(AngleUnit.DEGREES);
////        currentTurnAngle += Math.PI/2;
////        currentTurnAngle = Math.toDegrees(currentTurnAngle);
////        telemetry.addData("START : Yaw (Z)", "%.2f Deg. (Heading)", currentTurnAngle + " Target Heading is: " );
////        telemetry.update();
//        try {
//            sleep(2000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
////        int direction = 1;
////        int time;
//        if (degrees < 180) {
//            turn.setDirection(CRServo.Direction.REVERSE);
//        } else {
//            turn.setDirection(CRServo.Direction.FORWARD);
//            degrees = degrees - 360;
//        }
//        while ((degrees - currentTurnAngle)  > 2 ) {
//            turn.setPower(.2);
//            currentTurnAngle = orientation.getYaw(AngleUnit.DEGREES);
////            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", currentTurnAngle + " Target Heading is: " );
////            telemetry.update();
//        }
//        turn.setPower(0);

//        turn.setPower(Math.abs(pServo));
//        if (-(degrees  - currentTurnAngle ) > -2) {
//            turn.setPower(0);
//        }
        int time = 0;
        time = (int) (Math.abs(degrees) * 4.1);
        turn.setPower(1);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        turn.setPower(0);
    }

    static double area(Pose p1, Pose p2, Pose p3)
    {
        return abs((p1.getX()*(p2.getY()-p3.getY()) + p2.getX()*(p3.getY()-p1.getY())+
                p3.getX()*(p1.getY()-p2.getY()))/2.0);
    }

    /* A function to check whether point P(x, y) lies
       inside the triangle formed by A(x1, y1),
       B(x2, y2) and C(x3, y3) */
    public boolean isInside(Pose currentPose)
    {

        // big triangle points
        Pose p1 = new Pose(0, 144);
        Pose p2 = new Pose(144, 144);
        Pose p3 =  new Pose(72, 72);

        //smal triangle points
        Pose st1 = new Pose(96, 0);
        Pose st2 = new Pose(48, 0);
        Pose st3 =  new Pose(72, 24);
        /* Calculate area of triangle ABC */
        double A = area (p1, p2, p3);

        /* Calculate area of triangle PBC */
        double A1 = area (currentPose, p2, p3);

        /* Calculate area of triangle PAC */
        double A2 = area (p1, currentPose, p3);

        /* Calculate area of triangle PAB */
        double A3 = area (p1, p2, currentPose);

        // calculate for smaller triangle

        double STA = area (st1, st2, st3);

        /* Calculate area of triangle PBC */
        double STA1 = area (currentPose, st2, st3);

        /* Calculate area of triangle PAC */
        double STA2 = area (st1, currentPose, st3);

        /* Calculate area of triangle PAB */
        double STA3 = area (st1, st2, currentPose);
        /* Check if sum of A1, A2 and A3 is same as A */
        return (A == A1 + A2 + A3) || (STA == STA1 + STA2 + STA3);
    }

    public boolean isIntakeFull(){
        double distance = intakeSensor.getDistance(DistanceUnit.CM);
        ElapsedTime runtime = new ElapsedTime();

        if (distance < DISTANCE_THRESHOLD && !intakeFull) {

            if (firstTimeBallDetected == -1) {
                firstTimeBallDetected = runtime.milliseconds();
            }

            if (runtime.milliseconds() - firstTimeBallDetected >= STUCK_TIME) {
                intakeFull = true;
                ballCount = MAX_BALLS;
            }

            if (!ballDetectedPreviously && ballCount < MAX_BALLS) {
                ballDistances[ballCount] = distance;
                ballCount++;
                ballDetectedPreviously = true;
            }

        } else {
            ballDetectedPreviously = false;
            firstTimeBallDetected = -1;
        }

        telemetry.addData("Distance (cm)", "%.2f", distance);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Intake Full", intakeFull);
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        for (int i = 0; i < ballCount && i < MAX_BALLS; i++) {
            telemetry.addData("Ball " + (i + 1) + " Distance", "%.2f cm", ballDistances[i]);
        }

        return intakeFull;
    }
    public static class LaunchParameters {
        public double LAUNCH_POWER = 0;
        public double WAIT_TIME = 0;
        public double LAUNCH_ANGLE = 0;

        public LaunchParameters(double launchPower, double waitTime, double launchAngle) {
            LAUNCH_POWER = launchPower;
            WAIT_TIME = waitTime;
            LAUNCH_ANGLE = launchAngle;
        }
    }

}