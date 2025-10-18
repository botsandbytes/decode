package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Test")
public class oldshooter extends LinearOpMode {

    private static final double ROBOT_POWER = 0.6;

    @Override
    public void runOpMode() {

        BBRobot robot = new BBRobot(hardwareMap, telemetry);
        robot.isTeleOp = true;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x; // strafe
            double y = gamepad1.left_stick_y; // forward/back (up is negative)

            if (Math.abs(x) < 0.2) x = 0;
            if (Math.abs(y) < 0.2) y = 0;

            if (y < 0) {
                robot.moveForward(ROBOT_POWER);
            } else if (y > 0) {
                robot.moveBackward(ROBOT_POWER);
            } else if (x > 0) {
                robot.moveRight(ROBOT_POWER);
            } else if (x < 0) {
                robot.moveLeft(ROBOT_POWER);
            } else {
                robot.stopMotors();
            }

            telemetry.addData("Stick Y", y);
            telemetry.addData("Stick X", x);
            telemetry.update();
        }
    }
}