package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Test Pedro Auto generated Paths", group = "BB Auto")
public class TestPedroPath extends LinearOpMode {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(117, 128, Math.toRadians(45));

    public void Paths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(117.000, 128.000), new Pose(96.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.000, 96.000),
                                new Pose(94.000, 81.000),
                                new Pose(120.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.000, 84.000),
                                new Pose(94.000, 81.000),
                                new Pose(96.000, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .setReversed()
                .build();
    }

//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        Paths();
//    }

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        Paths();
        waitForStart();
        follower.followPath(Path1, true);
        follower.followPath(Path2, true);
        follower.followPath(Path3, true);
        follower.update();
    }
}
