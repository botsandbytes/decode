package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.intakeLaunch;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Opposite Auto", group = "BB Auto")
public class redOppositeAuto extends OpMode {

    boolean pickupLine2 = true;
    double shootPower = 0.91;
    int waitTimeForLaunch = 5000;
    double transferPower = .12;
    intakeLaunch intakeL ;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(87, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(87, 21, Math.toRadians(67));
    private final Pose pickup1Pose = new Pose(96, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseEnd = new Pose(132, 36, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(125, 11, Math.toRadians(0));
    private final Pose pickup2PoseEnd = new Pose(132, 11, Math.toRadians(0));
    private final Pose pickup2PoseEnd2 = new Pose(125, 8.5, Math.toRadians(0));
    private final Pose pickup2PoseEnd3 = new Pose(132.5, 8.5, Math.toRadians(0));// Highest (First Set) of Artifacts from the Spike Mark.

    // alternate curved path for pickup2
    private final Pose CurvePickup2Pose = new Pose(120, 45, Math.toRadians(270));
    private final Pose CurvePickup2PoseEnd = new Pose(134, 9, Math.toRadians(270));
    private final Pose CurvePickup2PoseReturn = new Pose(109, 28, Math.toRadians(270));

//    private final Pose scorePose2 = new Pose(96, 96, Math.toRadians(45));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup3Pose = new Pose(94, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PoseEnd = new Pose(134, 58, Math.toRadians(0));
    private final Pose pickup4Pose = new Pose(133, 26, Math.toRadians(0));
    private final Pose pickup5Pose = new Pose(133, 42, Math.toRadians(0));
    //94,36
    private Path scorePreload;
    private PathChain grabPickup1, grabPickup1Start, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, grabPickup5, scorePickup5;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose,  pickup1PoseEnd))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseEnd, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        //**** Straight Line Pick up code for Picking up corner balls
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose,  pickup2PoseEnd))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2PoseEnd.getHeading())
                .addPath(new BezierLine(pickup2PoseEnd,  pickup2PoseEnd2))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), pickup2PoseEnd2.getHeading())
                .addPath(new BezierLine(pickup2PoseEnd2,  pickup2PoseEnd3))
                .setLinearHeadingInterpolation(pickup2PoseEnd2.getHeading(), pickup2PoseEnd3.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .build();

//        //**** Curved  Pick up code for Picking up corner balls
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, CurvePickup2Pose, CurvePickup2PoseEnd))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), CurvePickup2PoseEnd.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(CurvePickup2PoseEnd, CurvePickup2PoseReturn, scorePose))
//                .setLinearHeadingInterpolation(CurvePickup2PoseEnd.getHeading(), scorePose.getHeading())
//                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose,  pickup3PoseEnd))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup4Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, scorePose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup5Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup5Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup5 = follower.pathBuilder()
                .addPath(new BezierLine(pickup5Pose, scorePose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                intakeL.powerOnLauncher(shootPower);
                setPathState(1);
                break;
            case 1:
//
//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
//                    /* Score Preload */
                    intakeL.takeShot(shootPower, waitTimeForLaunch);
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
                    intakeL.powerOnLauncher(shootPower);
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score pick up1 */
                    intakeL.stopIntake();
                    intakeL.takeShot(shootPower, waitTimeForLaunch);
                    intakeL.stopLauncher();
                    intakeL.runIntake(1, transferPower);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, .6,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intakeL.runIntake(1, transferPower);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intakeL.powerOnLauncher(shootPower);
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    intakeL.stopIntake();
                    intakeL.takeShot(shootPower, waitTimeForLaunch);
                    intakeL.stopLauncher();
                    if (pickupLine2) {
                        intakeL.runIntake(1, transferPower);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup3, true);
                    } else {
                        intakeL.runIntake(1, transferPower);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup4, true);
                    }
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    if (pickupLine2) {
                        /* Grab Sample */
                        intakeL.runIntake(1, transferPower);
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        intakeL.powerOnLauncher(shootPower);
                        follower.followPath(scorePickup3, true);
                    } else {
                        intakeL.runIntake(1, transferPower);
                        intakeL.powerOnLauncher(shootPower);
                        follower.followPath(scorePickup4, true);
                    }
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                        /* Score Sample */
                    intakeL.stopIntake();
                    intakeL.takeShot(shootPower, waitTimeForLaunch);
                    intakeL.stopLauncher();
                    intakeL.runIntake(1, transferPower);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup5, .6,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intakeL.runIntake(1, transferPower);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intakeL.powerOnLauncher(shootPower);
                    follower.followPath(scorePickup5, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    intakeL.stopIntake();
                    intakeL.takeShot(shootPower, waitTimeForLaunch);
                    intakeL.stopLauncher();
                    setPathState(10);
                }
                break;
            case 10:
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
        intakeL.setHoodLongShotPosition();

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