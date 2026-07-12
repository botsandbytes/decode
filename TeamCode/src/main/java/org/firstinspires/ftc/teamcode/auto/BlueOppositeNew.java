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
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.ConfigLoader;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Autonomous(name = "Blue Opposite NEW", group = "Blue Auto")
public class BlueOppositeNew extends OpMode {
    public static long drinkWaitTime = 1250;
    public static double shootWaitTime = 2450;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Follower follower;
    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;
    public static final double launchPower = 0.86;
    private final double transferPower = 0.12;

    private final Pose startPose = new Pose(57, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(59, 20, Math.toRadians(111));
    private final Pose drinkPoseCP = new Pose(50, 50, Math.toRadians(140));
    private final Pose drinkPoseEnd = new Pose(13, 60.5, Math.toRadians(140));
    private final Pose pickup2PoseCP = new Pose(60, 68, Math.toRadians(180));
    private final Pose pickup2PoseEnd = new Pose(12, 58, Math.toRadians(180));
    private final Pose pickup3PoseCP = new Pose(60, 40, Math.toRadians(180));
    private final Pose pickup3PoseEnd = new Pose(12, 36, Math.toRadians(180));
    private final Pose pickup4PoseCP = new Pose(50, 20, Math.toRadians(190));
    private final Pose pickup4PoseEnd = new Pose(12, 10, Math.toRadians(190));
    private final Pose parkPose = new Pose(40, 20, Math.toRadians(111));

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

        gatePark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading(), .1)
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup4PoseCP, pickup4PoseEnd ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4PoseEnd.getHeading(), .1)
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
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3PoseEnd.getHeading(), 0.3)
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0 -> {
                // go to score preload location
                shooter.powerOnLauncher(launchPower+0.02);
                follower.followPath(scorePreload);
                setPathState(1);
            }
            case 1 -> {
                // score preload & go to pick up line 2
                if (!follower.isBusy()) {
                    // score preload
                    if (!shooter.isShooting()) {
                        shooter.startShooting();
                    }
                    shooter.takeShot(launchPower, intake);

                    // stop shooting and go for drink pick up 1
                    if (shooter.getShootingDuration() > (shootWaitTime + 500)) {
                        shooter.stopShooting();
                        shooter.runShooterRaw(launchPower / 2);
                        intake.run(1, transferPower);
                        follower.followPath(grabPickup2);
                        setPathState(2);
                    }
                }
            }
            case 2 -> {
                // go to score pose for line 2
                if (!follower.isBusy()) {
                    shooter.powerOnLauncher(launchPower+0.02);
                    follower.followPath(scorePickup2);
                    setPathState(5);
                }
            }
            case 3 -> {
                // score line 2 & go to drink gate start round 1
                if (follower.atPose(scorePose, 1, 1)) {
                    intake.stop();
                    // score preload
                    if (!shooter.isShooting()) {
                        shooter.startShooting();
                    }
                    shooter.takeShot(launchPower, intake);

                    // stop shooting and go for drink pick up 1
                    if (shooter.getShootingDuration() > shootWaitTime) {
                        shooter.stopShooting();
                        shooter.runShooterRaw(launchPower / 2);
                        intake.run(1, transferPower);
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
                    shooter.powerOnLauncher(launchPower);
                    follower.followPath(drinkPickupScore);
                    setPathState(5);
                }
            }
            case 5 -> {
                // score drink 1  & go to drink gate start round 2
                if (!follower.isBusy()) {
                    intake.stop();
                    if (!shooter.isShooting()) {
                        shooter.startShooting();
                    }
                    shooter.takeShot(launchPower, intake);

                    if (shooter.getShootingDuration() > shootWaitTime) {
                        shooter.stopShooting();
                        shooter.runShooterRaw(launchPower / 2);
                        intake.run(1, transferPower);
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
                    shooter.powerOnLauncher(launchPower+0.02);
                    follower.followPath(drinkPickupScore);
                    setPathState(7);
                }
            }
            case 7 -> {
                // score the drink 2 and go to line 1 for pick up
                if (!follower.isBusy()) {
                    intake.stop();
                    if (!shooter.isShooting()) {
                        shooter.startShooting();
                    }
                    shooter.takeShot(launchPower, intake);

                    if (shooter.getShootingDuration() > (shootWaitTime-500)) {
                        shooter.stopShooting();
                        shooter.runShooterRaw(launchPower / 2);
                        intake.run(1, transferPower);
                        shooter.powerOnLauncher(launchPower);
                        follower.followPath(grabPickup3, true);
                        setPathState(8);
                    }
                }
            }
            case 8 -> {
                // go to score pose for line 1
                if (!follower.isBusy()) {
                    intake.run(1, transferPower);
                    shooter.powerOnLauncher(launchPower);
                    follower.followPath(scorePickup3);
                    setPathState(9);
                }
            }
            case 9 -> {
                // score line 1 & go to line 3
                if (!follower.isBusy()) {
                    intake.stop();
                    turret.updateTurret(follower.getPose());
                    // score line 1
                    if (!shooter.isShooting()) {
                        shooter.startShooting();
                    }
                    shooter.takeShot(launchPower, intake);

                    // stop shooting and go for drink pick up 3
                    if (shooter.getShootingDuration() > shootWaitTime) {
                        shooter.stopShooting();
                        shooter.runShooterRaw(launchPower / 2);
                        intake.run(1, transferPower);
                        shooter.powerOnLauncher(launchPower);
                        follower.followPath(grabPickup4, true);
                        setPathState(10);
                    }
                }
            }
            case 10 -> {
                // go to score pose for line 3
                if (!follower.isBusy()) {
                    intake.run(1, transferPower);
                    follower.followPath(scorePickup4);
                    shooter.powerOnLauncher(launchPower+0.02);
                    setPathState(11);
                }
            }
            case 11 -> {
                // score the line 3 and go to line 1 for park
                if (!follower.isBusy()) {
                    intake.stop();
                    turret.updateTurret(follower.getPose());
                    if (!shooter.isShooting()) {
                        shooter.startShooting();
                    }
                    shooter.takeShot(launchPower, intake);

                    if (shooter.getShootingDuration() > shootWaitTime) {
                        shooter.stopShooting();
                        follower.followPath(gatePark, true);
                        setPathState(12);
                    }
                }
            }
            case 12 -> {
                if (!follower.isBusy()) {
                    intake.stop();
                    shooter.stopShooting();
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
                follower.getPose().getHeading(), Math.toRadians(turret.getCurrentTurnAngle()), GOAL_X, GOAL_Y);
        follower.update();
        autonomousPathUpdate();

        turret.setTargetTurnAngle(Math.toDegrees(follower.getHeading()));
        turret.updateTurret(follower.getPose());
        if (opmodeTimer.getElapsedTime() > 28500) {
            shooter.stopShooting();
            follower.followPath(gatePark, true);
        }
        blackboard.put("BLUE_POSE", follower.getPose());

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
        blackboard.clear();
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
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, telemetry, follower);
        turret.setInitialHeading(startPose.getHeading());
        Shooter.minTransferThreashhold = 0.95;
        turret.setGoal(GOAL_X, GOAL_Y);
        buildPaths();
        follower.setStartingPose(startPose);
        shooter.setShooterPIDFCoefficients();
        shooter.setHoodLongShotPosition();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
