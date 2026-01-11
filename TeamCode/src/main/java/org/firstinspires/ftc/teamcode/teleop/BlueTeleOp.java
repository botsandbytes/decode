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
    private final Pose scorePose = new Pose(57, 21, Math.toRadians(68)).mirror();
    private boolean automatedDrive = false;
    private boolean isTurning = false;
    private Pose holdPose;
    private LaunchParameters currentLaunchParams;

    private boolean isRotating = false;

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
        intakeLauncher = new IntakeLauncher(hardwareMap, telemetry, follower);
        intakeLauncher.setGoal(GOAL_X, GOAL_Y);
        intakeLauncher.setInitialHeading(startPose.getHeading());
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
            double yInput = Math.max(-0.5, Math.min(0.5, Math.pow(-gamepad1.left_stick_y, 3)));
            double xInput = Math.max(-0.5, Math.min(0.5, Math.pow(-gamepad1.left_stick_x, 3)));
            double rInput = Math.max(-0.5, Math.min(0.5, Math.pow(-gamepad1.right_stick_x, 3)));

            double[] robotCentric = BorderPatrol.adjustDriveInput(follower.getPose(), follower.getPose().getX(), follower.getPose().getY(), xInput, yInput, rInput);
            follower.setTeleOpDrive(yInput, xInput, rInput, false);
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
            intakeLauncher.runShooterRaw(0.6);
        }
    }

    private void handleLauncher() {
        Pose currentPose = follower.getPose();
        currentLaunchParams = intakeLauncher.calculateLaunchParameters(currentPose);

        // Aiming Logic
        // In AUTO_SHOOT_MODE: right trigger does full sequence
        // In two-button mode: gamepad2.x aims, then right trigger shoots
        boolean shouldAim;
        if (IntakeLauncher.AUTO_SHOOT_MODE) {
            shouldAim = gamepad2.right_trigger > 0.5;
        } else {
            shouldAim = gamepad2.x;
        }

        if (shouldAim && !intakeLauncher.isShooting() && !isRotating && !isTurning && Sentinel.isLaunchAllowed(follower.getPose())) {
            // Set hood position based on launch power
            if (currentLaunchParams.launchPower() > 0.7) {
                intakeLauncher.setHoodLongShotPosition();
            } else {
                intakeLauncher.setHoodPosition(0);
            }

            intakeLauncher.setTargetTurnAngle(currentLaunchParams.launchAngle());
            isTurning = true;
            holdPose = follower.getPose();
        }

        if ((gamepad1.xWasPressed() || isRotating) && Sentinel.isLaunchAllowed(follower.getPose())) {
            isRotating = intakeLauncher.updateTurn(currentPose, currentLaunchParams.launchAngle());
            automatedDrive = isRotating || intakeLauncher.isShooting();

            if (!isRotating && !gamepad1.xWasPressed()) {
                intakeLauncher.setTargetTurnAngle(currentLaunchParams.launchAngle());
                isTurning = true;
            }
        }

        // Emergency Stop Launcher - MUST be checked first to override everything else
        if (gamepad2.left_trigger > 0.5) {
            stopShootingSequence();
            intakeLauncher.stopLauncher(); // Ensure shooter is completely stopped
            isTurning = false; // Stop turret tracking
            isRotating = false; // Stop robot rotation
        }
        // Auto rev-up logic: Keep shooter spinning at target velocity if it's already on
        else if (intakeLauncher.getShooterPower() > 0 && !intakeLauncher.isShooting()) {
            intakeLauncher.powerOnLauncher(currentLaunchParams.launchPower());
        }

        if (gamepad1.yWasPressed()) {
            isRotating = false;
            isTurning = false; // Allow cancelling turret aim manually
        }

        if (isTurning) {
            intakeLauncher.updateTurret(currentPose);
            if (intakeLauncher.isTurnDone()) {
                // Keep isTurning = true to maintain tracking/alignment

                // Auto-start shooting in AUTO_SHOOT_MODE when turret is done and trigger is held
                if (IntakeLauncher.AUTO_SHOOT_MODE && gamepad2.right_trigger > 0.5 && !intakeLauncher.isShooting()) {
                    startShootingSequence();
                }
            }
        }

        // Shooting Trigger
        // In AUTO_SHOOT_MODE: handled automatically after aiming
        // In two-button mode: right trigger manually starts shooting after aiming with X
        if (!IntakeLauncher.AUTO_SHOOT_MODE && gamepad2.right_trigger > 0.5 && !intakeLauncher.isShooting() && !isTurning) {
            startShootingSequence();
        }

        // Manual Shoot Trigger
        if (gamepad2.dpadUpWasPressed()) {
            startShootingSequence();
        }

        // Auto Drive Override
        if (gamepad2.dpadLeftWasPressed()) {
            automatedDrive = true;
            follower.holdPoint(new Pose(37.497382198952884, 31.929319371727757, -Math.PI / 2).mirror());
        }

        if (gamepad1.right_trigger > 0.5 && !follower.isBusy()) {
            follower.holdPoint(scorePose);
        }

        if (gamepad2.dpadRightWasPressed() || gamepad1.dpadRightWasPressed()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }


        if (intakeLauncher.isShooting()) {
            intakeLauncher.setTargetTurnAngle(currentLaunchParams.launchAngle());
            intakeLauncher.updateTurret(currentPose);
            intakeLauncher.updateShootingLogic(currentLaunchParams.launchPower(), currentPose);

            if (intakeLauncher.getShootingDuration() > currentLaunchParams.waitTime() || !Sentinel.isLaunchAllowed(follower.getPose())) {
                stopShootingSequence();
                // Return drive control immediately after shooting completes
                automatedDrive = false;
                follower.startTeleopDrive();
            }
        }

        // Manual Power On
        if (gamepad2.yWasPressed()) {
            intakeLauncher.runShooterRaw(0.6);
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
                follower.getPose().getHeading(), Math.toRadians(intakeLauncher.getCurrentTurnAngle()), GOAL_X, GOAL_Y);
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
