package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.BorderPatrol;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;

import java.util.List;

@TeleOp(name = "Border Patrol Test", group = "Test")
public class BorderPatrolTest extends OpMode {

    private Follower follower;
    private FieldManager field;
    private List<LynxModule> allHubs;

    private Pose startPose = new Pose(72, 72, 0);

    @Override
    public void init() {
        // Initialize Field Manager
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        // Initialize Hardware (Follower)
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        List<DcMotorEx> motors = hardwareMap.getAll(DcMotorEx.class);
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Clear bulk cache
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        follower.update();

        handleDrive();
        drawField();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Convert Pedro Pose to FTC Pose2D
        Pose pedroPose = follower.getPose();
        Pose2D currentPose = new Pose2D(
                DistanceUnit.INCH,
                pedroPose.getX(),
                pedroPose.getY(),
                AngleUnit.RADIANS,
                pedroPose.getHeading()
        );

        // Adjust inputs using BorderPatrol
        // Passing 0 for current velocity as it seems unused or we don't have it easily
        double[] adjusted = BorderPatrol.adjustDriveInput(
                currentPose,
                0.0, 0.0,
                strafe, forward, turn
        );

        // BorderPatrol returns [strafe, forward, turn]
        // Follower expects [forward, strafe, turn]
        follower.setTeleOpDrive(adjusted[1], adjusted[0], adjusted[2], true);

        telemetry.addData("Pos", "X:%.1f Y:%.1f H:%.2f", pedroPose.getX(), pedroPose.getY(), pedroPose.getHeading());
        telemetry.addData("Input", "F:%.2f S:%.2f T:%.2f", forward, strafe, turn);
        telemetry.addData("Output", "F:%.2f S:%.2f T:%.2f", adjusted[1], adjusted[0], adjusted[2]);
        
        // Debug info
        if (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1) {
            double inputMag = Math.sqrt(forward*forward + strafe*strafe);
            double outputMag = Math.sqrt(adjusted[1]*adjusted[1] + adjusted[0]*adjusted[0]);
            telemetry.addData("Magnitude", "In:%.2f Out:%.2f", inputMag, outputMag);
            if (inputMag > 0.1 && outputMag < 0.05) {
                telemetry.addData("WARNING", "Movement blocked!");
            }
        }
    }

    private void drawField() {
        if (field != null) {
            DrawingUtil.drawBorderPatrolZones(field);
            DrawingUtil.drawRobotOnField(field, follower.getPose().getX(), follower.getPose().getY(),
                    follower.getPose().getHeading(), 0, 0);
        }
    }
}

