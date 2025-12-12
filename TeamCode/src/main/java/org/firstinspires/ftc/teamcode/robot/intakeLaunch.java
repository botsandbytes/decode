package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intakeLaunch {

    private final Pose goalTargetPose = new Pose(138.0, 132.0, Math.PI / 4.0);
    private DcMotorEx intakeF, intakeM, shooter;
    private Servo hood;
    private Servo turn;
    private final Telemetry telemetry;
    public static double TRAIN_RPM_PERCENT = 1;
    public static double LAUNCH_VELOCITY = .7;
    public static int MAX_RPM = 1500;
    public static double INTAkE_TRANSFER_POWER = 1;
    double long_position = .35;

    public intakeLaunch(HardwareMap hardwareMap, Telemetry tel) {
        telemetry = tel;

        //Intake Motors
        intakeF = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeM = hardwareMap.get(DcMotorEx.class, "intakeMid");
        hood = hardwareMap.get(Servo.class, "hood");
        turn = hardwareMap.get(Servo.class, "turn");

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
        setTurnPosition(0.5);

    }

    public void runIntake(double intakePower, double transferPower) {
        telemetry.addData("Setting intake power to", intakePower);
        telemetry.addData("Setting transfer power to", transferPower);
        intakeF.setPower(intakePower);
        intakeM.setPower(transferPower);
        telemetry.addData("intake Front velocity ", intakeF.getVelocity());
        telemetry.update();
    }

    public void stopIntake() {
        telemetry.addData("Shutting down: ", "intake");
        telemetry.update();
        intakeM.setPower(0);
        intakeF.setPower(0);
    }

    public void takeShot(double launchPower, double waitTime) {
        telemetry.addData("Launching balls with power ", launchPower);
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        double targetVel = MAX_RPM * launchPower;
        while (runtime.milliseconds() < waitTime) {
            shooter.setVelocity(targetVel);
            // start feeding the ball as soon as launcher velocity reaches 90% of the target to reduce the transfer time
            if (Math.abs(shooter.getVelocity()) > MAX_RPM * (0.9 * launchPower)) {
                telemetry.addData("start the intake during shot ", shooter.getVelocity());
                telemetry.update();
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
        telemetry.update();
        double targetVel = MAX_RPM * launchPower * TRAIN_RPM_PERCENT;
        shooter.setVelocity(targetVel);
    }

    public void stopLauncher() {
        telemetry.addData("Shutting down: ", "Launcher");
        telemetry.update();
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
        telemetry.update();
        double deltaX = goalTargetPose.getX() - CurrentPose.getX();
        double deltaY = goalTargetPose.getY() - CurrentPose.getY();
        // Uses Math.sqrt() and Math.pow() or simply multiplication
        double distanceToGoal = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
//        double distanceToGoal = Math.hypot(goalTargetPose.getX() - CurrentPose.getX(), goalTargetPose.getY() - CurrentPose.getY());
        telemetry.addData("Distance is ", distanceToGoal);
        telemetry.update();
        return distanceToGoal;
    }

    public double getAngeleToGoal(Pose CurrentPose) {
        telemetry.addData("calculating angle for ", CurrentPose.getX() + ", " + CurrentPose.getY());
        telemetry.update();
        double angleRadians = Math.atan2(goalTargetPose.getY() - CurrentPose.getY(), goalTargetPose.getX() - CurrentPose.getX());
        telemetry.addData("Angle is ", angleRadians);
        telemetry.update();
//        return Math.toDegrees(angleRadians);
        return angleRadians;
    }

    public LaunchParameters calculateLaunchParameters(Pose CurrentPose) {
        telemetry.addData("calculating launch parameters for ", CurrentPose.getX() + ", " + CurrentPose.getY());
        telemetry.update();
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
            waitTime = distanceToGoal * 50;
        }

        LP = new LaunchParameters(launchPower, waitTime, angleToGoal);
        return LP;

    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public void setHoodLongShotPosition() {
        hood.setPosition(long_position);
    }

    public void setTurnPosition(double position) {
        turn.setPosition(position);
    }

    static double area(Pose p1, Pose p2, Pose p3)
    {
        return Math.abs((p1.getX()*(p2.getY()-p3.getY()) + p2.getX()*(p3.getY()-p1.getY())+
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
    public class LaunchParameters {
        public double LAUNCH_POWER = 0;
        public double WAIT_TIME = 0;
        public double LAUNCH_ANGLE = 0;

        LaunchParameters(double launchPower, double waitTime, double launchAngle) {
            LAUNCH_POWER = launchPower;
            WAIT_TIME = waitTime;
            LAUNCH_ANGLE = launchAngle;
        }
    }

}