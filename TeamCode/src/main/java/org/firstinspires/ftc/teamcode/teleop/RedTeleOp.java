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
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.LaunchParameters;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;
import org.firstinspires.ftc.teamcode.utilities.ConfigLoader;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;

import java.util.List;

@Configurable
@TeleOp(name = "TeleOp RED", group = "!")
public class RedTeleOp extends OpMode {
    public static double GOAL_X = 128.5;
    public static double GOAL_Y = 135;

    public static double MAXSPEED = 0.6;

    private Follower follower;
    private VisionUtil vision;
    private FieldManager field;
    private List<LynxModule> allHubs;
    private TelemetryManager telemetryM;

    // Robot Subsystems
    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    private PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> driverGamepad;
    private PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> operatorGamepad;

    // State
    private final Pose startPose = new Pose(87, 8, Math.toRadians(90));

    public static final Pose scorePose = Turret.AlignPose(61, 21, GOAL_X+2.5, GOAL_Y);
    public static final Pose drinkPose = new Pose(129, 60.5, Math.toRadians(42));
    public static final Pose parkPose = new Pose(37.5, 32, Math.toRadians(270));
    private boolean automatedDrive = false;
    private boolean isTurning = false;
    private Pose holdPose;
    private LaunchParameters currentLaunchParams;

    private boolean isRotating = false;

    private double turn;

    @Override
    public void init() {
        initializeField();
        initializeHardware();
        initializeSubsystems();
        Casablanca.reset();
        Casablanca.CURRENT_ALLIANCE = Casablanca.Alliance.RED;
        Pose blackboardPose = (Pose) blackboard.get("RED_POSE");
        if (blackboardPose != null) {
            follower.setStartingPose(blackboardPose);
        } else {
            follower.setStartingPose(startPose);
        }

        driverGamepad = Pasteurized.gamepad1();
        operatorGamepad = Pasteurized.gamepad2();

        shooter.setShooterPIDFCoefficients();
        turret.setInitialHeading(follower.getHeading());
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

        double drivetrainTolerance = ConfigLoader.getDouble("caching.drivetrain_tolerance");
        CachingDcMotorEx lf = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        CachingDcMotorEx lb = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftBack"));
        CachingDcMotorEx rf = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        CachingDcMotorEx rb = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBack"));

        lf.setCachingTolerance(drivetrainTolerance);
        lb.setCachingTolerance(drivetrainTolerance);
        rf.setCachingTolerance(drivetrainTolerance);
        rb.setCachingTolerance(drivetrainTolerance);

        hardwareMap.put("leftFront", lf);
        hardwareMap.put("leftBack", lb);
        hardwareMap.put("rightFront", rf);
        hardwareMap.put("rightBack", rb);

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void initializeSubsystems() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, telemetry, follower);
        turret.setGoal(GOAL_X, GOAL_Y);
        turret.setInitialHeading(startPose.getHeading());
        vision = new VisionUtil();
        vision.initAprilTag(hardwareMap, true);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.setInitialHeading(follower.getHeading());
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

        blackboard.put("RED_POSE", follower.getPose());
    }

    private void clearBulkCache() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    private void handleDrive() {
        if (!automatedDrive) {
            double yInput = Math.clamp(Math.pow(-driverGamepad.leftStickY().state(), 3), -MAXSPEED, MAXSPEED);
            double xInput = Math.clamp(Math.pow(-driverGamepad.leftStickX().state(), 3), -MAXSPEED, MAXSPEED);
            double rInput = Math.clamp(Math.pow(-driverGamepad.rightStickX().state(), 3), -MAXSPEED, MAXSPEED);

            double[] robotCentric = Casablanca.adjustDriveInput(follower.getPose(), follower.getVelocity(), xInput, yInput, rInput);
            follower.setTeleOpDrive(robotCentric[1], robotCentric[0], robotCentric[2], false);
        } else if (holdPose != null && shooter.isShooting()) {
            follower.holdPoint(holdPose);
        }
    }

    private void handleIntake() {
        if (operatorGamepad.a().onTrue()) {
            intake.run(1.0, 0.1);
        }

        if (operatorGamepad.b().onTrue()) {
            intake.stop();
            shooter.runShooterRaw(0.6);
        }
    }

    private void handleLauncher() {
        Pose currentPose = follower.getPose();
        currentLaunchParams = shooter.REDcalculateLaunchParameters(currentPose, GOAL_X, GOAL_Y);

        boolean shouldAim = Shooter.AUTO_SHOOT_MODE ? (operatorGamepad.rightTrigger().state() > 0.5) : operatorGamepad.x().state();

        if (shouldAim && !shooter.isShooting() && !isRotating && !isTurning && Sentinel.isLaunchAllowed(follower.getPose())) {
            if (currentLaunchParams.launchPower() > 0.7) {
                shooter.setHoodLongShotPosition();
            } else {
                shooter.setHoodPosition(0);
            }

            turn = currentLaunchParams.launchAngle();
            isTurning = true;
            holdPose = follower.getPose();
        }

        if ((driverGamepad.x().onTrue() || isRotating) && Sentinel.isLaunchAllowed(follower.getPose())) {
            isRotating = turret.updateTurn(currentPose, currentLaunchParams.launchAngle());
            automatedDrive = isRotating || shooter.isShooting();

            if (!isRotating && !driverGamepad.x().onTrue()) {
                turn = currentLaunchParams.launchAngle();
                isTurning = true;
            }
        }

        if (operatorGamepad.leftTrigger().state() > 0.5) {
            stopShootingSequence();
            shooter.stop();
            isTurning = false;
            isRotating = false;
        } else if (shooter.getShooterPower() > 0 && !shooter.isShooting()) {
            shooter.powerOnLauncher(currentLaunchParams.launchPower());
        }

        if (driverGamepad.y().onTrue()) {
            isRotating = false;
            isTurning = false;
        }

        if (isTurning) {
            turret.updateTurn(currentPose, turn);
            if (turret.isTurnDone()) {
                if (Shooter.AUTO_SHOOT_MODE && operatorGamepad.rightTrigger().state() > 0.5 && !shooter.isShooting()) {
                    startShootingSequence();
                }
            }
        }

        if (!Shooter.AUTO_SHOOT_MODE && operatorGamepad.rightTrigger().state() > 0.5 && !shooter.isShooting() && !isTurning) {
            startShootingSequence();
        }

        if (operatorGamepad.dpadUp().onTrue()) {
            startShootingSequence();
        }

        if (driverGamepad.dpadLeft().onTrue()) {
            automatedDrive = true;
            follower.holdPoint(parkPose);
        }

        if (driverGamepad.rightTrigger().state() > 0.5 && !follower.isBusy()) {
            shooter.setHoodLongShotPosition();
            follower.holdPoint(scorePose);
        }

        if (driverGamepad.leftTrigger().state() > 0.5 && !follower.isBusy()) {
            follower.holdPoint(drinkPose);
        }

        if (operatorGamepad.dpadRight().onTrue() || driverGamepad.dpadRight().onTrue()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        if (shooter.isShooting()) {
            turn = currentLaunchParams.launchAngle();
            shooter.updateShootingLogic(currentLaunchParams.launchPower(), currentPose, intake, turret, telemetry);

            if (shooter.getShootingDuration() > currentLaunchParams.waitTime() || !Sentinel.isLaunchAllowed(follower.getPose())) {
                stopShootingSequence();
                automatedDrive = false;
                follower.startTeleopDrive();
            }
        }

        if (operatorGamepad.y().onTrue()) {
            shooter.runShooterRaw(0.6);
        }
    }

    private void startShootingSequence() {
        isTurning = false;
        shooter.startShooting();
        automatedDrive = true;
        holdPose = follower.getPose();
    }

    private void stopShootingSequence() {
        shooter.stopShooting();
        automatedDrive = false;
        follower.startTeleopDrive();
    }

    private void handleVision() {
        if (operatorGamepad.dpadDown().state()) {
            Pose visionPose = vision.updateAprilTagPose();
            if (vision.isTagFound()) {
                follower.setPose(visionPose);
                telemetryM.addLine("Field Pose is: X: " + visionPose.getX() + " Y: " + visionPose.getY() + "Heading: " + visionPose.getHeading());
                telemetry.addLine("Camera Pose UPDATED");
                telemetry.addLine("Field Pose is: X: " + visionPose.getX() + " Y: " + visionPose.getY() + "Heading: " + visionPose.getHeading());
                telemetry.update();
            } else {
                telemetryM.addData("Vision", "No Tag Found");
            }
            vision.stopStreaming();
        }
    }

    private void drawField() {
        DrawingUtil.drawRobotOnField(field, follower.getPose().getX(), follower.getPose().getY(),
                follower.getPose().getHeading(), Math.toRadians(turret.getCurrentTurnAngle()), GOAL_X, GOAL_Y);
        DrawingUtil.drawCasablancaZones(field);
    }

    private void updateTelemetry() {
        telemetryM.addData("Pose", follower.getPose());
        if (currentLaunchParams != null) {
            telemetryM.addData("Target Angle", currentLaunchParams.launchAngle());
            telemetryM.addData("Launch Power", currentLaunchParams.launchPower());
        }
        telemetryM.addData("Current Heading", follower.getHeading());
        telemetryM.addData("Turret Angle", turret.getCurrentTurnAngle());
        telemetryM.addData("Is Shooting", shooter.isShooting());
        telemetryM.addData("Automated Drive", automatedDrive);
        telemetryM.update();
    }
}
