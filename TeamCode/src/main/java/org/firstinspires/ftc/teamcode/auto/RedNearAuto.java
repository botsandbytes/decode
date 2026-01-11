package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.teleop.RedTeleOp.GOAL_X;
import static org.firstinspires.ftc.teamcode.teleop.RedTeleOp.GOAL_Y;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.IntakeLauncher;
import org.firstinspires.ftc.teamcode.robot.LaunchParameters;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;

@Configurable
@Autonomous(name = "Red Near Auto", group = "Red Auto")
public class RedNearAuto extends OpMode {

    public static boolean openGateAfterPickup1 = false;
    public static boolean openGateAfterPickup2 = false;
    public static boolean pickupLine3 = true;

    private IntakeLauncher intakeLauncher;
    private Follower follower;
    private Timer pathTimer;
    private Timer opmodeTimer;

    private int pathState;
    private final double launchPower = 0.67;
    private final double transferPower = 0.12;

    private final Pose startPose = new Pose(117, 128, Math.toRadians(45));
    private final Pose scorePose = new Pose(90, 90, Math.toRadians(45));

    private final Pose pickup1Pose = new Pose(100, 84, Math.toRadians(0));
    private final Pose pickup1PoseEnd = new Pose(126, 84, Math.toRadians(0));
    private final Pose gateAfterPose1 = new Pose(115, 76, Math.toRadians(0));
    private final Pose gateAfterPose1End = new Pose(122, 76, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(96, 96, Math.toRadians(45));
    private final Pose pickup2Pose = new Pose(94, 60, Math.toRadians(0));
    private final Pose pickup2PoseEnd = new Pose(132, 60, Math.toRadians(0));
    private final Pose pickup2PoseReturn = new Pose(125, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(96, 40, Math.toRadians(0));
    private final Pose pickup3PoseEnd = new Pose(130, 36, Math.toRadians(0));
    private final Pose pickup3PoseReturn = new Pose(125, 36, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private FieldManager field;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, pickup1PoseEnd))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1PoseEnd.getHeading())
                .build();

        if (openGateAfterPickup1) {
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1PoseEnd, gateAfterPose1))
                    .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), gateAfterPose1.getHeading())
                    .addPath(new BezierLine(gateAfterPose1, gateAfterPose1End))
                    .setLinearHeadingInterpolation(gateAfterPose1.getHeading(), gateAfterPose1End.getHeading())
                    .addPath(new BezierLine(gateAfterPose1End, gateAfterPose1))
                    .setLinearHeadingInterpolation(gateAfterPose1End.getHeading(), gateAfterPose1.getHeading())
                    .addPath(new BezierLine(gateAfterPose1, scorePose2))
                    .setLinearHeadingInterpolation(gateAfterPose1.getHeading(), scorePose2.getHeading())
                    .build();
        } else {
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1PoseEnd, scorePose2))
                    .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose2.getHeading())
                    .build();
        }

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, pickup2PoseEnd))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2PoseEnd.getHeading())
                .build();

        if (openGateAfterPickup2) {
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2PoseEnd, pickup2PoseReturn))
                    .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), pickup2PoseReturn.getHeading())
                    .addPath(new BezierLine(pickup2PoseReturn, gateAfterPose1))
                    .setLinearHeadingInterpolation(pickup2PoseReturn.getHeading(), gateAfterPose1.getHeading())
                    .addPath(new BezierLine(gateAfterPose1, gateAfterPose1End))
                    .setLinearHeadingInterpolation(gateAfterPose1.getHeading(), gateAfterPose1End.getHeading())
                    .addPath(new BezierLine(gateAfterPose1End, gateAfterPose1))
                    .setLinearHeadingInterpolation(gateAfterPose1End.getHeading(), gateAfterPose1.getHeading())
                    .addPath(new BezierLine(gateAfterPose1, scorePose))
                    .setLinearHeadingInterpolation(gateAfterPose1.getHeading(), scorePose.getHeading())
                    .build();
        } else {
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2PoseEnd, pickup2PoseReturn))
                    .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), pickup2PoseReturn.getHeading())
                    .addPath(new BezierLine(pickup2PoseReturn, scorePose))
                    .setLinearHeadingInterpolation(pickup2PoseReturn.getHeading(), scorePose.getHeading())
                    .build();
        }

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose, pickup3PoseEnd))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3PoseEnd.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseEnd, pickup3PoseReturn))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), pickup3PoseReturn.getHeading())
                .addPath(new BezierLine(pickup3PoseReturn, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseReturn.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0 -> {
                follower.followPath(scorePreload);
                intakeLauncher.powerOnLauncher(launchPower);
                setPathState(1);
            }
            case 1 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > 2500) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(grabPickup1);
                        setPathState(2);
                    }
                }
            }
            case 2 -> {
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    follower.followPath(scorePickup1, true);
                    intakeLauncher.powerOnLauncher(0.65);
                    setPathState(3);
                }
            }
            case 3 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.updateShootingLogic(0.65, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > 2800) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                    }
                }
            }
            case 4 -> {
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    follower.followPath(scorePickup2, true);
                    intakeLauncher.powerOnLauncher(launchPower);
                    setPathState(5);
                }
            }
            case 5 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > 3400) {
                        intakeLauncher.stopShooting();
                        if (pickupLine3) {
                            intakeLauncher.runIntake(1, transferPower);
                            follower.followPath(grabPickup3, true);
                        }
                        setPathState(6);
                    }
                }
            }
            case 6 -> {
                if (!follower.isBusy()) {
                    if (pickupLine3) {
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(scorePickup3, true);
                        intakeLauncher.powerOnLauncher(launchPower);
                    }
                    setPathState(7);
                }
            }
            case 7 -> {
                if (!follower.isBusy()) {
                    if (pickupLine3) {
                        if (!intakeLauncher.isShooting()) {
                            intakeLauncher.startShooting();
                        }
                        intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                        if (intakeLauncher.getShootingDuration() > 3500) {
                            intakeLauncher.stopShooting();
                            setPathState(8);
                        }
                    } else {
                        setPathState(8);
                    }
                }
            }
            case 8 -> {
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
            }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        DrawingUtil.drawRobotOnField(field, follower.getPose().getX(), follower.getPose().getY(),
                follower.getPose().getHeading(), Math.toRadians(intakeLauncher.getCurrentTurnAngle()), GOAL_X, GOAL_Y);
        follower.update();
        autonomousPathUpdate();

        LaunchParameters lp = intakeLauncher.calculateLaunchParameters(follower.getPose());
        intakeLauncher.setTargetTurnAngle(lp.launchAngle());
        intakeLauncher.updateTurret(follower.getPose());

        Pose currentPose = follower.getPose();
        blackboard.put("POSE_X", currentPose.getX());
        blackboard.put("POSE_Y", currentPose.getY());
        blackboard.put("POSE_HEADING", currentPose.getHeading());

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        intakeLauncher = new IntakeLauncher(hardwareMap, telemetry, follower);
        intakeLauncher.setInitialHeading(startPose.getHeading());

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
