package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.BBPinPointDrive;
import org.firstinspires.ftc.teamcode.robot.intakeLaunch.LaunchParameters;
import org.firstinspires.ftc.teamcode.robot.intakeLaunch;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PedroTeleOp extends OpMode {
    private Follower follower;
    private VisionUtil vision;
    private FieldManager field;
    public static Pose startingPose = new Pose(72, 72, Math.toRadians(0)); //See ExampleAuto to understand how to use this
    private Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0);
    private boolean automatedDrive;
    private boolean launchReady = false, takeShot = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    intakeLaunch intakeL;
    BBPinPointDrive bbDrive;
    LaunchParameters lp = new LaunchParameters(0,0,0);
//    private final Pose scorePose = new Pose(87, 21.5, Math.toRadians(71));

    @Override
    public void init() {
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intakeL = new intakeLaunch(hardwareMap, telemetry);
        vision = new VisionUtil();
        vision.initAprilTag(hardwareMap, true);
        bbDrive = new BBPinPointDrive(hardwareMap, telemetry);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(96, 96))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void init_loop() {
        Pose2D pose = vision.updateAprilTagPose();
        if (vision.isTagFound()) startPose = pose;

        telemetry.addData("Tag Found", vision.isTagFound() ? "YES" : "NO");
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
//        intakeL.getAngeleToGoal(scorePose);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
//        DrawingUtil.drawRobotOnField(field, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() )
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing - A for automated drive, b for manual drive
        if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.leftBumperWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode controls on gamepad 2
        if (gamepad2.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad2.xWasPressed()) {
            slowModeMultiplier += 0.2;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.2;
        }

        // Gamepad 1 (should be moved to Gamepad 2 later) - intake A - Run ; B-STOP
        if (gamepad1.aWasPressed()){ //&& !intakeL.isIntakeFull()
            intakeL.runIntake(1, .1);
        }
        if (gamepad1.bWasPressed()){
            intakeL.stopIntake();
        }
        Pose pose = follower.getPose();
        lp = intakeL.calculateLaunchParameters(pose);
        telemetryM.addData("target", lp.LAUNCH_ANGLE);
        telemetryM.addData("current", follower.getHeading());
        telemetryM.addData("ha", follower.getHeadingError());

        if (gamepad1.x && !follower.isBusy() && !launchReady){
            if(lp.LAUNCH_POWER > 0.7){
                intakeL.setHoodLongShotPosition();
            }else{
                intakeL.setHoodPosition(0);
            }
//            follower.turnTo(lp.LAUNCH_ANGLE);
            bbDrive.turn(lp.LAUNCH_ANGLE);
//            Path path = new Path(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX() + 0.001, follower.getPose().getY() + 0.0001, lp.LAUNCH_ANGLE)));
//            path.setLinearHeadingInterpolation(follower.getHeading(), lp.LAUNCH_ANGLE);
//            follower.followPath(path);
//            intakeL.takeShot(lp.LAUNCH_POWER, lp.WAIT_TIME);
            launchReady = true;
            takeShot = true;
            automatedDrive = true;
        }
//        if (launchReady && !follower.isBusy() && automatedDrive) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
        if (gamepad1.dpad_up && takeShot){
            intakeL.takeShot(lp.LAUNCH_POWER, lp.WAIT_TIME);
            takeShot = false;
            launchReady = false;
        }
        if (gamepad1.y){
            intakeL.stopLauncher();
//            intakeL.powerOnLauncher(.65);
        }

        if (gamepad1.dpadRightWasPressed()){
            intakeL.getAngeleToGoal(follower.getPose());
            intakeL.getPoseDistance(follower.getPose());
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.addData("Pose in launch area", intakeL.isInside(follower.getPose()));
        telemetryM.update();
    }
}