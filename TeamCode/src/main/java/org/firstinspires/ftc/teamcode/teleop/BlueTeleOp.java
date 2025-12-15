package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.BBPinPointDrive;
import org.firstinspires.ftc.teamcode.robot.intakeLaunch;
import org.firstinspires.ftc.teamcode.robot.intakeLaunch.LaunchParameters;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp BLUE", group = "!")
public class BlueTeleOp extends OpMode {
    public static double GOAL_X = 144-RedTeleOp.GOAL_X;
    public static double GOAL_Y = RedTeleOp.GOAL_Y;

    private Follower follower;
    private VisionUtil vision;
    private FieldManager field;
    private List<LynxModule> allHubs;

    // Default start if AprilTag fails
    public static Pose startingPose = new Pose(72, 72, Math.toRadians(0));
    // Pose container for AprilTag updates
    private Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0);

    private boolean automatedDrive;
    private boolean launchReady = false, takeShot = false;
    private Supplier<PathChain> pathChain; // Warning: This is null!
    private TelemetryManager telemetryM;

    private Pose hold;
    intakeLaunch intakeL;
    BBPinPointDrive bbDrive;
    LaunchParameters lp = new LaunchParameters(0,0,0);

    private boolean turning = false;

    @Override
    public void init() {
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if (blackboard.containsKey("POSE_X")) {
            double x = (double) blackboard.get("POSE_X");
            double y = (double) blackboard.get("POSE_Y");
            double h = (double) blackboard.get("POSE_HEADING");
            startingPose = new Pose(x, y, h);
        }

        follower = Constants.createFollower(hardwareMap);
        // Do not update follower here yet, wait for start or init_loop updates

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intakeL = new intakeLaunch(hardwareMap, telemetry);
        vision = new VisionUtil();
        vision.initAprilTag(hardwareMap, true);
        bbDrive = new BBPinPointDrive(hardwareMap, telemetry);

        intakeLaunch.goalX = GOAL_X;
        intakeLaunch.goalY = GOAL_Y;
        intakeLaunch.degrees = 90;
    }

    @Override
    public void init_loop() {
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        // FIXED: Apply the vision-corrected start pose to the follower
        follower.setStartingPose(startingPose);

        follower.startTeleopDrive();
        intakeLaunch.initial = follower.getHeading();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        follower.update();

        DrawingUtil.drawRobotOnField(field, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), GOAL_X, GOAL_Y);
        DrawingUtil.drawBorderPatrolZones(field);

        if (!automatedDrive) {
            double yInput = Math.clamp(Math.pow(gamepad1.left_stick_y, 3), -0.5, 0.5);
            double xInput = Math.clamp(Math.pow(gamepad1.left_stick_x, 3), -0.5, 0.5);
            double rInput = Math.clamp(Math.pow(-gamepad1.right_stick_x, 3), -0.5, 0.5);

            // adjusted[0] = Strafe, adjusted[1] = Forward
            // setTeleOpDrive(forward, strafe, turn, fieldCentric)
            follower.setTeleOpDrive(yInput, xInput, rInput, false);
        }

        // Intake controls
        if (gamepad2.aWasPressed()){
            intakeL.runIntake(1, .1);
        }
        if (gamepad2.bWasPressed()){
            intakeL.stopIntake();
        }

        Pose pose = follower.getPose();
        telemetryM.addData("pose", pose);
        lp = intakeL.calculateLaunchParameters(pose);
        telemetryM.addData("target", lp.LAUNCH_ANGLE);
        telemetryM.addData("current", follower.getHeading());

        if (gamepad2.x){
            if(lp.LAUNCH_POWER > 0.7){
                intakeL.setHoodLongShotPosition();
            } else {
                intakeL.setHoodPosition(0);
            }
            intakeLaunch.degrees = lp.LAUNCH_ANGLE;
            turning = true;
            hold = follower.getPose();
        }

        if (turning) {
//            intakeL.setTurnPosition();
            if (intakeLaunch.done) {
                turning = false;
                launchReady = true;
                takeShot = true;
            }
        }

        if (gamepad2.dpadUpWasPressed()) {
            if(lp.LAUNCH_POWER > 0.7){
                intakeL.setHoodLongShotPosition();
            } else {
                intakeL.setHoodPosition(0);
            }
                turning = false;
                launchReady = true;
                takeShot = true;
                intakeLaunch.shooting = true;
                intakeLaunch.runtime.reset();
                automatedDrive = true;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            automatedDrive = true;
            follower.holdPoint(new Pose(144-38.123197903014415, 33.40498034076016, Math.PI/2));
        }

        if (gamepad2.dpadRightWasPressed()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        telemetryM.addData("done", intakeLaunch.done);

        if (gamepad2.right_trigger > 0.5 && takeShot){
            intakeLaunch.shooting = true;
            intakeLaunch.runtime.reset();
            automatedDrive = true;
        }

        intakeL.takeShot(lp.LAUNCH_POWER, lp.WAIT_TIME);

        if (intakeLaunch.shooting && hold != null) {
            follower.holdPoint(hold);
        }

        if (intakeLaunch.runtime.milliseconds() > lp.WAIT_TIME && intakeLaunch.shooting) {
            intakeLaunch.shooting = false;
            launchReady = false;
            takeShot = false;
            automatedDrive = false;
            follower.startTeleopDrive(); // Re-enable teleop after shot
        }

        telemetryM.addData("intakeL", intakeLaunch.degrees);
        telemetryM.addData("currentYAW", intakeLaunch.currentTurnAngle);

        if (gamepad2.left_trigger > 0.5){
            intakeL.stopLauncher();
            intakeLaunch.shooting = false;
            launchReady = false;
            takeShot = false;
            automatedDrive = false;
            follower.startTeleopDrive();
        }


        if (gamepad2.dpad_down){
            Pose2D visionPose = vision.updateAprilTagPose();
            if (vision.isTagFound()) {
                Pose pedroPose = PoseConverter.pose2DToPose(visionPose, InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                Pose currentPose = follower.getPose();
                Pose newPose = new Pose(pedroPose.getX(), pedroPose.getY(), currentPose.getHeading());
                follower.setPose(newPose);
            }
            vision.stopStreaming();
        }

        if (gamepad2.yWasPressed()) {
            intakeL.powerOnLauncher(0.6);
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.update();
    }
}
