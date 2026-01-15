package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.teleop.RedTeleOp.GOAL_X;
import static org.firstinspires.ftc.teamcode.teleop.RedTeleOp.GOAL_Y;
import static java.lang.Thread.sleep;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
@Autonomous(name = "Red Auto NEW", group = "Red Auto")
public class RedAutoNew extends OpMode {
    public static double slowVelocity = 5;
    public static long drinkWaitTime = 1250;
    public static double shootWaitTime = 1250;
    private IntakeLauncher intakeLauncher;
    private Follower follower;
    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;
    public static final double launchPower = 0.7;
    private final double transferPower = 0.12;

    private final Pose startPose = new Pose(117, 128, Math.toRadians(45));
    private final Pose scorePose = new Pose(88, 80, Math.toRadians(52));
    private final Pose drinkPose = new Pose(120, 55, Math.toRadians(45));
    private final Pose drinkPoseEnd = new Pose(129, 60.5, Math.toRadians(42));
    private final Pose pickup1Pose = new Pose(100, 84, Math.toRadians(0));
    private final Pose pickup1PoseEnd = new Pose(123, 84, Math.toRadians(0));
    private final Pose gateAfterPose1 = new Pose(103, 76, Math.toRadians(0));
    private final Pose gateAfterPose1End = new Pose(122, 76, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(96, 96, Math.toRadians(45));
    private final Pose pickup2Pose = new Pose(94, 60, Math.toRadians(0));
    private final Pose pickup2PoseEnd = new Pose(130, 60, Math.toRadians(0));
    private final Pose pickup2PoseReturn = new Pose(120, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(88, 25, Math.toRadians(0));
    private final Pose pickup3PoseEnd = new Pose(130, 36, Math.toRadians(0));
    private final Pose pickup3PoseReturn = new Pose(125, 36, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, drinkPickupStart, drinkPickup, drinkPickupScore, grabPickup2, scorePickup2, grabPickup3, scorePickup3, gatePark;
    private FieldManager field;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        drinkPickupStart = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, drinkPose))
                .addPath(new BezierCurve(scorePose, new Pose(96, 72), drinkPoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), drinkPoseEnd.getHeading())
                .build();

        drinkPickup = follower.pathBuilder()
                .addPath(new BezierLine(drinkPose, drinkPoseEnd))
                .setLinearHeadingInterpolation(drinkPose.getHeading(), drinkPoseEnd.getHeading())
                .setVelocityConstraint(slowVelocity)
                .build();

        drinkPickupScore = follower.pathBuilder()
//                .addPath(new BezierLine(drinkPoseEnd, drinkPose))
//                .setLinearHeadingInterpolation(drinkPoseEnd.getHeading(), drinkPose.getHeading())
//                .addPath(new BezierLine(drinkPose, scorePose))
//                .setLinearHeadingInterpolation(drinkPose.getHeading(), scorePose.getHeading())
                .addPath(new BezierCurve(drinkPoseEnd, new Pose(96, 72), scorePose ))
                .setLinearHeadingInterpolation(drinkPoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .addPath(new BezierLine(pickup1Pose, pickup1PoseEnd))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1PoseEnd.getHeading())
                .addPath(new BezierCurve(scorePose, pickup1Pose, pickup1PoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseEnd.getHeading(), .1)
                .build();

        gatePark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1Pose, gateAfterPose1 ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateAfterPose1.getHeading(), .1)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .addPath(new BezierLine(pickup2Pose, pickup2PoseEnd))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2PoseEnd.getHeading())
                .addPath(new BezierCurve(scorePose, new Pose(84, 55), pickup2PoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseEnd.getHeading(), 0.5)
                .build();


        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2PoseEnd, pickup2PoseReturn))
//                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), pickup2PoseReturn.getHeading())
//                .addPath(new BezierLine(pickup2PoseReturn, scorePose))
//                .setLinearHeadingInterpolation(pickup2PoseReturn.getHeading(), scorePose.getHeading())
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(84, 55), scorePose ))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose2, pickup3Pose))
//                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
//                .addPath(new BezierLine(pickup3Pose, pickup3PoseEnd))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3PoseEnd.getHeading())
                .addPath(new BezierCurve(scorePose, pickup3Pose, pickup3PoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3PoseEnd.getHeading(), 0.5)
                .build();

        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3PoseEnd, pickup3PoseReturn))
//                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), pickup3PoseReturn.getHeading())
//                .addPath(new BezierLine(pickup3PoseReturn, scorePose))
//                .setLinearHeadingInterpolation(pickup3PoseReturn.getHeading(), scorePose.getHeading())
                .addPath(new BezierCurve(pickup3PoseEnd, pickup3Pose,  scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0 -> {
                // go to score preload location
                intakeLauncher.powerOnLauncher(launchPower-0.02);
                follower.followPath(scorePreload);
                setPathState(1);
            }
            case 1 -> {
                // score preload & pick up line 2
                if (follower.atPose(scorePose, 1, 1)) {
                    intakeLauncher.stopIntake();
                    // score preload
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.takeShot(launchPower);
//                    intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                    // stop shooting and go for drink pick up 1
                    if (intakeLauncher.getShootingDuration() > shootWaitTime) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(grabPickup2, true);
                        setPathState(2);
                    }
                }
            }
            case 2 -> {
                // go to score pose for line 2
                if (!follower.isBusy()) {
//                    intakeLauncher.runIntake(1, transferPower);
                    intakeLauncher.powerOnLauncher(launchPower);
                    follower.followPath(scorePickup2);
                    setPathState(3);
                }
            }
            case 3 -> {
                // score line 2 & go to drink gate start round 1
                if (follower.atPose(scorePose, 1, 1)) {
                    intakeLauncher.stopIntake();
                    // score preload
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.takeShot(launchPower);

                    // stop shooting and go for drink pick up 1
                    if (intakeLauncher.getShootingDuration() > shootWaitTime) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(drinkPickupStart);
                        setPathState(4);
                    }
                }
            }
            case 4 -> {
                // drink balls and go to score pose
                if (!follower.isBusy()) {
                    // drink
                    try {
                        sleep(drinkWaitTime);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    // go to score pose
                    intakeLauncher.powerOnLauncher(launchPower);
                    follower.followPath(drinkPickupScore);
                    setPathState(5);
                }
            }
            case 5 -> {
                // score drink 1  & go to drink gate start round 2
                if (follower.atPose(scorePose, 1, 1)) {
                    intakeLauncher.stopIntake();
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.takeShot(launchPower);

                    if (intakeLauncher.getShootingDuration() > shootWaitTime) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
//                        intakeLauncher.powerOnLauncher(launchPower);
//                        follower.followPath(grabPickup1, true);
//                        setPathState(8);
                        follower.followPath(drinkPickupStart);
                        setPathState(6);
                    }
                }
            }
            case 6 -> {
                // drink round 2 balls and go to score pose
                if (!follower.isBusy()) {
                    // drink
                    try {
                        sleep(drinkWaitTime);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    // go to score pose
                    intakeLauncher.powerOnLauncher(launchPower);
                    follower.followPath(drinkPickupScore);
                    setPathState(7);
                }
            }
            case 7 -> {
                // score the drink 2 and go to drink start for round 3
                if (follower.atPose(scorePose, 1, 1)) {
                    intakeLauncher.stopIntake();
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.takeShot(launchPower);
//                    intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > shootWaitTime) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
                        intakeLauncher.powerOnLauncher(launchPower);
                        follower.followPath(grabPickup1, true);
                        setPathState(8);
                    }
                }
            }
            case 8 -> {
                // go to score pose for line 1
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    intakeLauncher.powerOnLauncher(launchPower);
                    follower.followPath(scorePickup1);
                    setPathState(9);
                }
            }
            case 9 -> {
                // score line 1 & go to line 3
                if (follower.atPose(scorePose, 1, 1)) {
                    // score line 3
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.takeShot(launchPower + 0.03);
//                    intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                    // stop shooting and go for drink pick up 1
                    if (intakeLauncher.getShootingDuration() > shootWaitTime) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(grabPickup3);
                        setPathState(10);
                    }
                }
            }
            case 10 -> {
                // go to score pose for line 1
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    follower.followPath(scorePickup3);
                    intakeLauncher.powerOnLauncher(launchPower);
                    setPathState(11);
                }
            }
            case 11 -> {
                // score the line 3 and go to line 1 for park
                if (follower.atPose(scorePose, 1, 1)) {
                    intakeLauncher.stopIntake();
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.takeShot(launchPower);
//                    intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                    if (intakeLauncher.getShootingDuration() > shootWaitTime) {
                        intakeLauncher.stopShooting();
//                        intakeLauncher.runIntake(1, transferPower);
                        follower.followPath(gatePark, true);
                        setPathState(12);
                    }
                }
            }
            case 12 -> {
                if (!follower.isBusy()) {
                    intakeLauncher.stopIntake();
                    intakeLauncher.stopShooting();
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
//        intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

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
        IntakeLauncher.minTransferThreashhold = 0.95;
        intakeLauncher.setGoal(GOAL_X, GOAL_Y);
        buildPaths();
        follower.setStartingPose(startPose);
        intakeLauncher.setShooterPIDFCoefficients();
    }

    @Override
    public void start() {
//        intakeLauncher.powerOnLauncher(launchPower);
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
