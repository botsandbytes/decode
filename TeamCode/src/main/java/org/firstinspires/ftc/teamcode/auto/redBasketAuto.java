package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.intakeLaunch;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Basket Auto", group = "BB Auto")
public class redBasketAuto extends OpMode {

    boolean openGateAfterPickup1 = false;
    boolean openGateAfterPickup2 = true;
    boolean pickupLine3 = true;
    intakeLaunch intakeL ;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    double launchPower = .68;
    double transferPower = .12;

    private final Pose startPose = new Pose(117, 128, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(90, 90, Math.toRadians(45));

    private final Pose pickup1Pose = new Pose(100, 84, Math.toRadians(0));
    private final Pose pickup1PoseEnd = new Pose(124, 84, Math.toRadians(0));// 128.5Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose gateAfterPose1 = new Pose(115,76, Math.toRadians(0));
    private final Pose gateAfterPose1End = new Pose(122,76, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(96, 96, Math.toRadians(45));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup2Pose = new Pose(94, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose pickup2PoseEnd = new Pose(130, 60, Math.toRadians(0));
    private final Pose pickup2PoseReturn = new Pose(125,60, Math.toRadians(0));
//94,36
    private final Pose pickup3Pose = new Pose(96, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose pickup3PoseEnd = new Pose(130, 36, Math.toRadians(0));
    private final Pose pickup3PoseReturn = new Pose(125,36, Math.toRadians(0));
    private Path scorePreload;
    private PathChain grabPickup1, grabPickup1Start, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;



    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

//        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose,  pickup1PoseEnd))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1PoseEnd.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        if (openGateAfterPickup1) {
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1PoseEnd, gateAfterPose1))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(gateAfterPose1, gateAfterPose1End))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(gateAfterPose1End, gateAfterPose1))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(gateAfterPose1, scorePose2))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();
        } else {
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1PoseEnd, scorePose2))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();
        }

//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose,  pickup2PoseEnd))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        if (openGateAfterPickup2) {
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2PoseEnd, pickup2PoseReturn))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(pickup2PoseReturn, gateAfterPose1))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(gateAfterPose1, gateAfterPose1End))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(gateAfterPose1End, gateAfterPose1))
                    .setLinearHeadingInterpolation(gateAfterPose1End.getHeading(), gateAfterPose1.getHeading())
                    .addPath(new BezierLine(gateAfterPose1, scorePose))
                    .setLinearHeadingInterpolation(gateAfterPose1.getHeading(), scorePose.getHeading())
                    .build();

        } else {
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2PoseEnd, pickup2PoseReturn))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(pickup2PoseReturn, scorePose))
                    .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                    .build();
        }

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose,  pickup3PoseEnd))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseEnd, pickup3PoseReturn))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(pickup3PoseReturn, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                intakeL.powerOnLauncher(launchPower);
                setPathState(1);
                break;
            case 1:
//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
//                    /* Score Preload */
                    intakeL.takeShot(launchPower, 2500);
                    intakeL.stopLauncher();
                    intakeL.runIntake(1, transferPower);
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intakeL.runIntake(1, transferPower);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    intakeL.powerOnLauncher(.65);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score pick up1 */
                    intakeL.stopIntake();
                    intakeL.takeShot(0.65, 2800);
                    intakeL.stopLauncher();
                    intakeL.runIntake(1, transferPower);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intakeL.runIntake(1, transferPower);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    intakeL.powerOnLauncher(launchPower);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    intakeL.stopIntake();
                    intakeL.takeShot(launchPower, 3400);
                    intakeL.stopLauncher();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    // pick up 3rd based on flag
                    if (pickupLine3) {
                        intakeL.runIntake(1, transferPower);
                        follower.followPath(grabPickup3, true);
                    }
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    // pick up 3rd based on flag
                    if (pickupLine3) {
                        /* Grab Sample */
                        intakeL.runIntake(1, transferPower);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup3, true);
                        intakeL.powerOnLauncher(launchPower);
                    }
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    // score 3rd based on flag
                    if (pickupLine3) {
                        /* Score Sample */
                        intakeL.stopIntake();
                        intakeL.takeShot(launchPower, 3500);
                        intakeL.stopLauncher();
                    }
                    setPathState(8);
                }
                break;

            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        intakeL = new intakeLaunch(hardwareMap, telemetry);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}