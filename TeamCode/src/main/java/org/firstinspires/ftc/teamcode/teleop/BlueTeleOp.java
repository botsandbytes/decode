package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.IntakeLauncher;
import org.firstinspires.ftc.teamcode.robot.LaunchParameters;
import org.firstinspires.ftc.teamcode.utilities.BorderPatrol;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;

import java.util.List;

@Configurable
@TeleOp(name = "TeleOp BLUE", group = "!")
public class BlueTeleOp extends OpMode {
    public static double GOAL_X = 144 - RedTeleOp.GOAL_X;
    public static double GOAL_Y = RedTeleOp.GOAL_Y;

    private Follower follower;
    private VisionUtil vision;
    private FieldManager field;
    private List<LynxModule> allHubs;
    private TelemetryManager telemetryM;

    // Robot Subsystems
    private IntakeLauncher intakeLauncher;

    // State
    private final Pose startPose = new Pose(72, 72, 0);
    private boolean automatedDrive = false;
    private boolean isTurning = false;
    private Pose holdPose;
    private LaunchParameters currentLaunchParams;

    @Override
    public void init() {
        initializeField();
        initializeHardware();
        initializeSubsystems();
    }

    private void initializeField() {
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    private void initializeHardware() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void initializeSubsystems() {
        intakeLauncher = new IntakeLauncher(hardwareMap, telemetry);
        intakeLauncher.setGoal(GOAL_X, GOAL_Y);
        intakeLauncher.setInitialHeading(0);

        vision = new VisionUtil();
        vision.initAprilTag(hardwareMap, true);
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        intakeLauncher.setInitialHeading(follower.getHeading());
    }

    @Override
    public void loop() {
        clearBulkCache();
        follower.update();

        handleDrive();
        handleIntake();
        handleLauncher();
        handleVision();

        drawField();
        updateTelemetry();
    }

    private void clearBulkCache() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    private void handleDrive() {
        if (!automatedDrive) {
            double yInput = Math.max(-0.7, Math.min(0.7, Math.pow(gamepad1.left_stick_y, 3)));
            double xInput = Math.max(-0.7, Math.min(0.7, Math.pow(-gamepad1.left_stick_x, 3)));
            double rInput = Math.max(-0.7, Math.min(0.7, Math.pow(-gamepad1.right_stick_x, 3)));

            double[] robotCentric = BorderPatrol.adjustDriveInput(follower.getPose(), follower.getPose().getX(), follower.getPose().getY(), xInput, yInput, rInput);
            follower.setTeleOpDrive(robotCentric[1], robotCentric[0], robotCentric[2]);
            follower.setTeleOpDrive(yInput, xInput, rInput, true);
        } else if (holdPose != null && intakeLauncher.isShooting()) {
            follower.holdPoint(holdPose);
        }
    }

    private void handleIntake() {
        if (gamepad2.aWasPressed()) {
            intakeLauncher.runIntake(1, 0.1);
        }
        if (gamepad2.bWasPressed()) {
            intakeLauncher.stopIntake();
        }
    }

    private void handleLauncher() {
        Pose currentPose = follower.getPose();
        currentLaunchParams = intakeLauncher.calculateLaunchParameters(currentPose);

        // Aiming
        if (gamepad2.x && Sentinel.isLaunchAllowed(follower.getPose())) {
            if (currentLaunchParams.launchPower() > 0.7) {
                intakeLauncher.setHoodLongShotPosition();
            } else {
                intakeLauncher.setHoodPosition(0);
            }

            intakeLauncher.setTargetTurnAngle(currentLaunchParams.launchAngle());
            isTurning = true;
            holdPose = follower.getPose();
        }

        if (isTurning) {
            intakeLauncher.updateTurret(Math.toDegrees(follower.getHeading()));
            if (intakeLauncher.isTurnDone()) {
                isTurning = false;
            }
        }

        // Manual Shoot Trigger
        if (gamepad2.dpadUpWasPressed()) {
            startShootingSequence();
        }
        // Auto Drive Override
        if (gamepad2.dpadLeftWasPressed()) {
            automatedDrive = true;
            follower.holdPoint(new Pose(38.12, 33.40, Math.PI / 2));
        }

        if (gamepad2.dpadRightWasPressed()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        // Shooting Logic
        if (gamepad2.right_trigger > 0.5 && !intakeLauncher.isShooting()) {
             startShootingSequence();
        }

        if (intakeLauncher.isShooting()) {
            intakeLauncher.setTargetTurnAngle(currentLaunchParams.launchAngle());
            intakeLauncher.updateTurret(Math.toDegrees(follower.getHeading()));
            intakeLauncher.updateShootingLogic(currentLaunchParams.launchPower());

            if (intakeLauncher.getShootingDuration() > currentLaunchParams.waitTime() || !Sentinel.isLaunchAllowed(follower.getPose())) {
                stopShootingSequence();
            }
        }

        // Emergency Stop Launcher
        if (gamepad2.left_trigger > 0.5) {
            stopShootingSequence();
        }

        // Manual Power On
        if (gamepad2.yWasPressed()) {
            intakeLauncher.powerOnLauncher(0.6);
        }
    }

    private void startShootingSequence() {
        isTurning = false;
        intakeLauncher.startShooting();
        automatedDrive = true;
        holdPose = follower.getPose();
    }

    private void stopShootingSequence() {
        intakeLauncher.stopShooting();
        automatedDrive = false;
        follower.startTeleopDrive();
    }

    private void handleVision() {
        if (gamepad2.dpad_down) {
            Pose2D visionPose = vision.updateAprilTagPose();
            if (vision.isTagFound()) {
                Pose pedroPose = PoseConverter.pose2DToPose(visionPose, InvertedFTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                Pose newPose = new Pose(pedroPose.getX(), pedroPose.getY(), follower.getHeading());
                follower.setPose(newPose);
            }
            else {
                telemetry.addData("Vision", "No Tag Found");
            }
            vision.stopStreaming();
        }
    }

    private void drawField() {
        DrawingUtil.drawRobotOnField(field, follower.getPose().getX(), follower.getPose().getY(),
                follower.getPose().getHeading(), GOAL_X, GOAL_Y);
        DrawingUtil.drawBorderPatrolZones(field);
    }

    private void updateTelemetry() {
        telemetryM.addData("Pose", follower.getPose());
        if (currentLaunchParams != null) {
            telemetryM.addData("Target Angle", currentLaunchParams.launchAngle());
            telemetryM.addData("Launch Power", currentLaunchParams.launchPower());
        }
        telemetryM.addData("Current Heading", follower.getHeading());
        telemetryM.addData("Turret Angle", intakeLauncher.getCurrentTurnAngle());
        telemetryM.addData("Is Shooting", intakeLauncher.isShooting());
        telemetryM.addData("Automated Drive", automatedDrive);
        telemetryM.update();
    }
}
