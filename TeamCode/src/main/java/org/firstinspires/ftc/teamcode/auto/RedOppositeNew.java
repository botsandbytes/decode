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
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;

@Configurable
@Autonomous(name = "Red Opposite NEW", group = "Red Auto")
public class RedOppositeNew extends OpMode {
    public static long drinkWaitTime = 1250;
    public static double shootWaitTime = 2000;
    private IntakeLauncher intakeLauncher;
    private Follower follower;
    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;
    public static final double launchPower = 0.85;
    private final double transferPower = 0.12;

    private final Pose startPose = new Pose(87, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(87, 21, Math.toRadians(68));
    private final Pose drinkPoseCP = new Pose(90, 50, Math.toRadians(42));
    private final Pose drinkPoseEnd = new Pose(129, 60.5, Math.toRadians(42));
    private final Pose pickup2PoseCP = new Pose(90, 60, Math.toRadians(0));
    private final Pose pickup2PoseEnd = new Pose(130, 60, Math.toRadians(0));
    private final Pose pickup3PoseCP = new Pose(90, 40, Math.toRadians(0));
    private final Pose pickup3PoseEnd = new Pose(130, 36, Math.toRadians(0));
    private final Pose pickup4PoseCP = new Pose(100, 84, Math.toRadians(0));
    private final Pose pickup4PoseEnd = new Pose(90, 15, Math.toRadians(0));
    private final Pose parkNearPose3 = new Pose(103, 60, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup4, scorePickup4, drinkPickupStart, drinkPickupScore, grabPickup2, scorePickup2, grabPickup3, scorePickup3, gatePark;
    private FieldManager field;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        drinkPickupStart = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, drinkPoseCP, drinkPoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), drinkPoseEnd.getHeading())
                .build();

        drinkPickupScore = follower.pathBuilder()
                .addPath(new BezierCurve(drinkPoseEnd, drinkPoseCP, scorePose ))
                .setLinearHeadingInterpolation(drinkPoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup4PoseCP, pickup4PoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4PoseEnd.getHeading(), .1)
                .build();

        gatePark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,parkNearPose3 ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkNearPose3.getHeading(), .1)
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup4PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2PoseCP, pickup2PoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseEnd.getHeading(), 0.5)
                .build();


        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, pickup2PoseCP, scorePose ))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup3PoseCP, pickup3PoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3PoseEnd.getHeading(), 0.5)
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0 -> {
                // go to score preload location
                intakeLauncher.powerOnLauncher(launchPower);
                follower.followPath(scorePreload);
                setPathState(1);
            }
            case 1 -> {
                // score preload & go to pick up line 2
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
                // score the drink 2 and go to line 1 for pick up
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
                        follower.followPath(grabPickup4, true);
                        setPathState(8);
                    }
                }
            }
            case 8 -> {
                // go to score pose for line 1
                if (!follower.isBusy()) {
                    intakeLauncher.runIntake(1, transferPower);
                    intakeLauncher.powerOnLauncher(launchPower);
                    follower.followPath(scorePickup4);
                    setPathState(9);
                }
            }
            case 9 -> {
                // score line 1 & go to line 3
                if (follower.atPose(scorePose, 1, 1)) {
                    intakeLauncher.stopIntake();
                    // score line 1
                    if (!intakeLauncher.isShooting()) {
                        intakeLauncher.startShooting();
                    }
                    intakeLauncher.takeShot(launchPower + 0.03);
//                    intakeLauncher.updateShootingLogic(launchPower, follower.getPose());

                    // stop shooting and go for drink pick up 3
                    if (intakeLauncher.getShootingDuration() > shootWaitTime) {
                        intakeLauncher.stopShooting();
                        intakeLauncher.runIntake(1, transferPower);
                        intakeLauncher.powerOnLauncher(launchPower);
                        follower.followPath(grabPickup3);
                        setPathState(10);
                    }
                }
            }
            case 10 -> {
                // go to score pose for line 3
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
