package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utilities.BorderPatrol;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Shooter Training")
@SuppressWarnings("unused")
@Configurable
public class train extends OpMode {

    public static double GOAL_X = -62.9921259843;
    public static double GOAL_Y = -GOAL_X;

    public static double TRAIN_HOOD_ANGLE = 0.5;
    public static double TRAIN_RPM_PERCENT = 1;
    public static int MAX_RPM = 1560;

    public static double INTAKE_L_POWER = 1.0;
    public static double INTAKE_M_POWER = 1.0;
    public static double INTAKE_L_POWER_3 = 1;

    private Robot robot;
    private VisionUtil vision;
    private FieldManager field;

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    private Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private boolean shooting = false;
    private double shooterTargetVelocity;

    private Follower follower;
    public static Pose startingPose;

    public enum DriveMode { ROBOT_CENTRIC, FIELD_CENTRIC }
    public static DriveMode CURRENT_DRIVE_MODE = DriveMode.ROBOT_CENTRIC;



    @Override
    public void init() {
        try {
            field = PanelsField.INSTANCE.getField();
            field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());

            robot = new Robot(hardwareMap);

            vision = new VisionUtil();
            vision.initAprilTag(hardwareMap, true);
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }

        follower = Constants.createFollower(hardwareMap);
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
        vision.stopStreaming();
        follower.setStartingPose(PoseConverter.pose2DToPose(startPose, InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE));
        follower.update();
        follower.startTeleopDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();

        double yInput = -gamepad1.left_stick_y;
        double xInput = -gamepad1.left_stick_x;
        double rInput = gamepad1.right_stick_x;

        double velX = robot.odo.getVelX(DistanceUnit.INCH);
        double velY = robot.odo.getVelY(DistanceUnit.INCH);

        Pose2D pose2d = PoseConverter.poseToPose2D(pose, InvertedFTCCoordinates.INSTANCE);

        double[] adjusted = BorderPatrol.adjustDriveInput(
                pose2d, velX, velY, xInput, yInput, rInput
        );

        follower.setTeleOpDrive(adjusted[1], adjusted[0], adjusted[2], CURRENT_DRIVE_MODE == DriveMode.ROBOT_CENTRIC);


        double poseX = pose2d.getX(DistanceUnit.INCH);
        double poseY = pose2d.getY(DistanceUnit.INCH);
        double headingRad = pose2d.getHeading(AngleUnit.RADIANS);

        double distanceToGoal = Math.hypot(GOAL_X - poseX, GOAL_Y - poseY);
        double angleToGoalRad = Math.atan2(GOAL_Y - poseY, GOAL_X - poseX);
        double relativeAngleDeg =
                normalizeAngle(Math.toDegrees(angleToGoalRad - headingRad));

        robot.setHoodPosition(TRAIN_HOOD_ANGLE);

        DrawingUtil.drawRobotOnField(field, poseX, poseY, headingRad, GOAL_X, GOAL_Y);
        DrawingUtil.drawBorderPatrolZones(field);

        handleIntakeAndShooter();

        telemetryM.addData("BP Alliance", BorderPatrol.CURRENT_ALLIANCE);
        telemetryM.addData("Dist", distanceToGoal);
        telemetryM.addData("Turret Ang", relativeAngleDeg);
        telemetryM.addData("Pose", String.format("%.1f, %.1f, %.1f°",
                poseX, poseY, Math.toDegrees(headingRad)));
        telemetryM.addData("shooter V", Math.abs(robot.getShooterVelocity()));
        telemetryM.addData("Shooter Target", shooterTargetVelocity);
        telemetryM.update(telemetry);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }


    private void handleIntakeAndShooter() {
        boolean intake;
        boolean shot = false;

        if (gamepad1.a && gamepad1.x) {
            intake = true;
            shooting = true;
        }
        else if (gamepad1.a) {
            intake = true;
            shooting = false;
            robot.setIntakeMidPower(0.1);
        }
        else if (gamepad1.x) {
            intake = false;
            shooting = true;
            shot = false;
            shooterTimer.reset();
        }
        else {
            intake = false;
            if (!shooting) {
                robot.setShooterVelocity(0);
                robot.setIntakePower(0);
            }
        }

        if (intake) {
            robot.setIntakeFrontPower(INTAKE_L_POWER);
        }

        if (shooting) {
            shooterTargetVelocity = MAX_RPM * 0.9 * TRAIN_RPM_PERCENT;
            robot.setShooterVelocity(shooterTargetVelocity);

            if (Math.abs(robot.getShooterVelocity()) > shooterTargetVelocity) {
                robot.setIntakeMidPower(INTAKE_M_POWER);
                robot.setIntakeFrontPower(INTAKE_L_POWER_3);
                shot = true;
            } else {
                robot.setIntakePower(0);
            }

            if (gamepad1.y) shooting = false;
        }
    }
}
