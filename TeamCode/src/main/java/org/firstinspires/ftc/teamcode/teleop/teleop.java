package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.intakeLaunch;
import org.firstinspires.ftc.teamcode.robot.intakeLaunch.LaunchParameters;
import org.firstinspires.ftc.teamcode.utilities.BorderPatrol;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;

import java.util.function.Supplier;

@TeleOp(name = "TeleOp")
@SuppressWarnings("unused")
@Configurable
public class teleop extends OpMode {

    public static double GOAL_X = 144;
    public static double GOAL_Y = 144;

    public static double TRAIN_HOOD_ANGLE = 0.5;
    public static double TRAIN_RPM_PERCENT = 1;
    public static int MAX_RPM = 1560;

    public static double INTAKE_L_POWER = 1.0;
    public static double INTAKE_M_POWER = 1.0;
    public static double INTAKE_L_POWER_3 = 1;

    public static double turnP = 4.0, turnI = 0, turnD = 0.2;

    private Robot robot;
    private VisionUtil vision;
    private FieldManager field;
    private intakeLaunch intakeL;

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    private final Pose startPose = new Pose(72, 72, 0);

    private boolean shooting = false;
    private double shooterTargetVelocity;

    private Follower follower;
    public static Pose startingPose;

    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;

    public enum DriveMode { ROBOT_CENTRIC, FIELD_CENTRIC }
    public static DriveMode CURRENT_DRIVE_MODE = DriveMode.ROBOT_CENTRIC;

    private boolean launchReady = false;
    private LaunchParameters lp = new LaunchParameters(0, 0, 0);

    // PID Variables
    private double targetHeading;
    private double lastErrorHeading = 0;
    private boolean isAligning = false;
    private boolean manualHoodLocked = false;

    public double turnPower;

    private double getTurnPower(double currentHeading, double targetHeading) {
        double error = normalizeAngle(Math.toDegrees(targetHeading - currentHeading));
        double derivative = error - lastErrorHeading;
        lastErrorHeading = error;
        return (error * turnP + derivative * turnD) / 100.0; // Scale down
    }

    @Override
    public void init() {
        try {
            field = PanelsField.INSTANCE.getField();
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

            robot = new Robot(hardwareMap);
            intakeL = new intakeLaunch(hardwareMap, telemetry);

            vision = new VisionUtil();
            vision.initAprilTag(hardwareMap, true);
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }

        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop() {
        Pose2D pose = vision.updateAprilTagPose();
        follower.setStartingPose(PoseConverter.pose2DToPose(pose, InvertedFTCCoordinates.INSTANCE));

        telemetry.addData("Tag Found", vision.isTagFound() ? "YES" : "NO");
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        // vision.stopStreaming();
        follower.setStartingPose(startPose);
        follower.update();
        follower.startTeleopDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();


        handleDrive();
        handleAutomatedDrive();

        Pose2D pose2d = PoseConverter.poseToPose2D(pose, InvertedFTCCoordinates.INSTANCE);

        double poseX = pose2d.getX(DistanceUnit.INCH);
        double poseY = pose2d.getY(DistanceUnit.INCH);
        double headingRad = pose2d.getHeading(AngleUnit.RADIANS);

        double distanceToGoal = Math.hypot(GOAL_X - poseX, GOAL_Y - poseY);
        double angleToGoalRad = Math.atan2(GOAL_Y - poseY, GOAL_X - poseX);
        double relativeAngleDeg =
                normalizeAngle(Math.toDegrees(angleToGoalRad - headingRad));

        // robot.setHoodPosition(TRAIN_HOOD_ANGLE); // Moved to handleIntakeAndShooter to avoid conflict

        DrawingUtil.drawRobotOnField(field, poseX, poseY, headingRad, GOAL_X, GOAL_Y);
        DrawingUtil.drawBorderPatrolZones(field);

        handleIntakeAndShooter();

        telemetryM.addData("BP Alliance", BorderPatrol.CURRENT_ALLIANCE);
        telemetryM.addData("Dist", distanceToGoal);
        telemetryM.addData("Turret Ang", relativeAngleDeg);
        telemetryM.addData("Pose", String.format("%.1f, %.1f, %.1f°",
                poseX, poseY, Math.toDegrees(headingRad)));
        telemetryM.addData("shooter V", Math.abs(robot.getShooterVelocity()));
        telemetryM.addData("Shooter Target", shooterTargetVelocity);

        // PedroTeleOp Telemetry
        telemetryM.addData("target", lp.LAUNCH_ANGLE);
        telemetryM.addData("current", follower.getHeading());
        telemetryM.addData("ha", follower.getHeading() - lp.LAUNCH_ANGLE);

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.addData("Pose in launch area", intakeL.isInside(follower.getPose()));
        telemetryM.update(telemetry);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void handleDrive() {
        if (!automatedDrive) {
            // PedroTeleOp uses negative right stick x for rotation
            double yInput = -gamepad1.left_stick_y;
            double xInput = -gamepad1.left_stick_x;
            double rInput = -gamepad1.right_stick_x;

            Pose2D pose2d = PoseConverter.poseToPose2D(follower.getPose(), InvertedFTCCoordinates.INSTANCE);

            double[] adjusted = BorderPatrol.adjustDriveInput(
                    pose2d, follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), xInput, yInput, rInput
            );

            follower.setTeleOpDrive(adjusted[1], adjusted[0], adjusted[2], CURRENT_DRIVE_MODE == DriveMode.ROBOT_CENTRIC);
        }
    }

    private void handleAutomatedDrive() {
        if (gamepad1.rightBumperWasPressed()) {
            //follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.leftBumperWasPressed() || !follower.isBusy())) {
            if (!launchReady) { // Don't interrupt launch turn unless explicit
                follower.startTeleopDrive();
                automatedDrive = false;
            }
        }
    }

    private void handleIntakeAndShooter() {
        // --- Manual State Updates ---
        boolean manualIntake = false;

        // Train.java style state machine
        if (gamepad1.a && gamepad1.x) {
            manualIntake = true;
            shooting = true;
        } else if (gamepad1.a) {
            manualIntake = true;
            shooting = false;
        } else if (gamepad1.x) {
            // manualIntake is already false
            if (!shooting) shooterTimer.reset();
            shooting = true;
        }

        if (gamepad1.y) {
            shooting = false;
            manualIntake = false;
            launchReady = false;
            isAligning = false;
            manualHoodLocked = false;
            intakeL.stopLauncher();
        }

        // Removed conflicting B-hold pathing; reserve B for intake control below

        // --- Automated Triggers ---

        // PedroTeleOp style automated shooting with intakeLaunch
        if (gamepad1.bWasPressed()) {
            intakeL.stopIntake();
            manualIntake = false;
        }

        // Automated Align (PID Turn + Hold Position)
        if (gamepad1.dpadUpWasPressed() && !launchReady) {
            Pose pose = follower.getPose();
            lp = intakeL.calculateLaunchParameters(pose);

            if (lp.LAUNCH_POWER > 0.7) {
                intakeL.setHoodLongShotPosition();
            } else {
                intakeL.setHoodPosition(0);
            }

            // Initialize PID Turn
            targetHeading = lp.LAUNCH_ANGLE;
            lastErrorHeading = 0;

            launchReady = true;
            isAligning = true;
            automatedDrive = true;
            manualHoodLocked = false;
        }

        // Automated Shoot (PedroTeleOp style)
        if (gamepad1.dpadRightWasPressed()) {
             intakeL.takeShot(lp.LAUNCH_POWER, lp.WAIT_TIME);
             launchReady = false; // Reset after shot
             isAligning = false;
             manualHoodLocked = false;
             robot.setHoodPosition(TRAIN_HOOD_ANGLE); // Reset hood
             follower.startTeleopDrive(); // Release PP lock
             automatedDrive = false;
        }

        if (gamepad1.dpadDownWasPressed()) {
            intakeL.stopLauncher();
            launchReady = false;
            isAligning = false;
            manualHoodLocked = false;
            robot.setHoodPosition(TRAIN_HOOD_ANGLE); // Reset hood
            follower.startTeleopDrive(); // Release PP lock
            automatedDrive = false;
        }

        // --- Actuation Arbitration ---

        if (launchReady) {
            if (isAligning) {
                Pose currentPose = follower.getPose();
                turnPower = getTurnPower(currentPose.getHeading(), targetHeading);

                // Check if aligned (e.g. within 1.5 degrees)
                double errorDegrees = normalizeAngle(Math.toDegrees(targetHeading - currentPose.getHeading()));
                if (Math.abs(errorDegrees) < 1.5) {
                    stop();
                    isAligning = false;
                    // Lock with PP
                    // follower.holdPoint(follower.getPose());
                } else {
                    // PID Turn, stop X/Y
                    follower.setTeleOpDrive(0, 0, turnPower, true);
                }
            }
            // If not aligning (and launchReady is true), we are holding via PP (followPath called above).

            // Allow manual intake override even during alignment
            if (manualIntake) {
                robot.setIntakeFrontPower(INTAKE_L_POWER);
            }
        } else {
            // Manual Mode

            if (!manualHoodLocked) {
                robot.setHoodPosition(TRAIN_HOOD_ANGLE);
                manualHoodLocked = true;
            }

            if (shooting) {
                shooterTargetVelocity = MAX_RPM * 0.9 * TRAIN_RPM_PERCENT;
                robot.setShooterVelocity(shooterTargetVelocity);

                if (Math.abs(robot.getShooterVelocity()) > shooterTargetVelocity) {
                    robot.setIntakeMidPower(INTAKE_M_POWER);
                    robot.setIntakeFrontPower(INTAKE_L_POWER_3);
                } else {
                    robot.setIntakePower(0);
                }
            } else if (manualIntake) {
                robot.setIntakeFrontPower(INTAKE_L_POWER);
                robot.setIntakeMidPower(0.1); // Keep mid intake slow for intake only
                robot.setShooterVelocity(0);
            } else {
                // Idle
                robot.setShooterVelocity(0);
                robot.setIntakePower(0);
            }
        }
    }
}
