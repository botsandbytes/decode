package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.teleop.BlueTeleOp.GOAL_X;
import static org.firstinspires.ftc.teamcode.teleop.BlueTeleOp.GOAL_Y;

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
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;

@Configurable
@Autonomous(name = "Blue Opposite Auto", group = "Blue Auto")
public class BlueOppositeAuto extends OpMode {

    public static boolean pickupLine2 = false;
    private final double shootPower = 0.905;
    private final int waitTimeForLaunch = 4500;
    private final double transferPower = 0.12;

    private IntakeLauncher intakeLauncher;
    private Follower follower;
    private Timer pathTimer;
    private Timer opmodeTimer;

    private int pathState;

    // Mirrored Poses
    private final Pose startPose = new Pose(87, 8, Math.toRadians(90)).mirror();
    private final Pose scorePose = new Pose(87, 21, Math.toRadians(70)).mirror();
    private final Pose pickup1Pose = new Pose(96, 36, Math.toRadians(0)).mirror();
    private final Pose pickup1PoseEnd = new Pose(132, 36, Math.toRadians(0)).mirror();
    private final Pose pickup2Pose = new Pose(125, 13, Math.toRadians(0)).mirror();
    private final Pose pickup2PoseEnd = new Pose(131, 13, Math.toRadians(0)).mirror();
    private final Pose pickup2PoseEnd2 = new Pose(125, 9.5, Math.toRadians(0)).mirror();
    private final Pose pickup2PoseEnd3 = new Pose(131, 9.5, Math.toRadians(0)).mirror();

    private final Pose pickup3Pose = new Pose(94, 60, Math.toRadians(0)).mirror();
    private final Pose pickup3PoseEnd = new Pose(134, 58, Math.toRadians(0)).mirror();
    private final Pose pickup4Pose = new Pose(133, 26, Math.toRadians(0)).mirror();
    private final Pose pickup5Pose = new Pose(133, 42, Math.toRadians(0)).mirror();

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, grabPickup5, scorePickup5;

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

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, pickup2PoseEnd))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2PoseEnd.getHeading())
                .addPath(new BezierLine(pickup2PoseEnd, pickup2PoseEnd2))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), pickup2PoseEnd2.getHeading())
                .addPath(new BezierLine(pickup2PoseEnd2, pickup2PoseEnd3))
                .setLinearHeadingInterpolation(pickup2PoseEnd2.getHeading(), pickup2PoseEnd3.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose, pickup3PoseEnd))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3PoseEnd.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup4Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4Pose.getHeading())
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, scorePose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup5Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup5Pose.getHeading())
                .build();

        scorePickup5 = follower.pathBuilder()
                .addPath(new BezierLine(pickup5Pose, scorePose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0 -> {
                follower.followPath(scorePreload);
                intakeLauncher.powerOnLauncher(shootPower);
                setPathState(1);
            }
            case 1 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.updateShootingLogic(shootPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > waitTimeForLaunch) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.stopLauncher();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(grabPickup2);
                        setPathState(2);
                    }
                }
            }
            case 2 -> {
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    intakeLauncher.powerOnLauncher(shootPower);
                    follower.followPath(scorePickup2, true);
                    setPathState(3);
                }
            }
            case 3 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.stopIntake();
                    intakeLauncher.updateShootingLogic(shootPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > waitTimeForLaunch) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.stopLauncher();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(grabPickup2, 0.6, true);
                        setPathState(4);
                    }
                }
            }
            case 4 -> {
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    intakeLauncher.powerOnLauncher(shootPower);
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
            }
            case 5 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.stopIntake();
                    intakeLauncher.updateShootingLogic(shootPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > waitTimeForLaunch) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.stopLauncher();
                        if (pickupLine2) {
                            intakeLauncher.runIntake(1, transferPower);
                            follower.followPath(grabPickup3, true);
                        } else {
                            intakeLauncher.runIntake(1, transferPower);
                            follower.followPath(grabPickup2, true);
                        }
                        setPathState(6);
                    }
                }
            }
            case 6 -> {
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    intakeLauncher.powerOnLauncher(shootPower);
                    if (pickupLine2) {
                        follower.followPath(scorePickup3, true);
                    } else {
                        follower.followPath(scorePickup2, true);
                    }
                    setPathState(7);
                }
            }
            case 7 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.stopIntake();
                    intakeLauncher.updateShootingLogic(shootPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > waitTimeForLaunch) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.stopLauncher();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(grabPickup2, 0.6, true);
                        setPathState(8);
                    }
                }
            }
            case 8 -> {
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    intakeLauncher.powerOnLauncher(shootPower);
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
            }
            case 9 -> {
                if (!follower.isBusy()) {
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.stopIntake();
                    intakeLauncher.updateShootingLogic(shootPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > waitTimeForLaunch) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.stopLauncher();
                        setPathState(10);
                    }
                }
            }
            case 10 -> {
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

        intakeLauncher.setTargetTurnAngle(Math.toDegrees(follower.getHeading()));
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
        intakeLauncher.setHoodLongShotPosition();
    }
}
