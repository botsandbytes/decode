package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.mecanumTeleop;

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Simple TeleOp")
@Configurable
public  class simpleTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        PinpointLocalizer drive = new PinpointLocalizer(hardwareMap, 2, beginPose);
        //PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        mecanumTeleop mecanum = new mecanumTeleop(hardwareMap, gamepad1);
        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        mecanum.drivetrain(),
                        teleopControls()
                )
        );
    }

    public class TeleopControl implements Action {

        int hangCount = 0;
        final double pi = Math.PI;

        Pose2d beginPose = new Pose2d(6, -33, -pi/2);
        PinpointLocalizer drive = new PinpointLocalizer(hardwareMap, 2, beginPose);
     //   PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        Pose2d specimentPickUpPose = new Pose2d(33, -55, pi/2);

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // GAMEPAD 1 Controls
//            drive.updatePoseEstimate();
            // Intake Claw Controls
            if (gamepad1.a) {

            }
            if (gamepad1.x) {

            }
            if (gamepad1.b) {
            }
            if (gamepad1.y) {

            }




            // GAMEPAD 2 Controls


            // Hang Claw Controls

            if (gamepad2.a) {

            }

            // intake claw rotate control

            if (gamepad2.y) {

            }
            if (gamepad2.b) {

            }
            if (gamepad2.x) {

            }
            return true;
        }
    }

    public Action teleopControls() {
        return new TeleopControl();
    }
}