package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Boolean.TRUE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumTeleop {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Gamepad gamepad1;

    public mecanumTeleop(HardwareMap hardwareMap, Gamepad gamepad1) {
        // Save the gamepad reference
        this.gamepad1 = gamepad1;

        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");

        // Reverse the left-side motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class drivetrain implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Calculate motor powers
            double denominator = Math.max(Math.pow(Math.abs(y) + Math.abs(x) + Math.abs(rx), 3), 1);
            double frontLeftPower = Math.pow(y + x + rx, 3) / denominator;
            double backLeftPower = Math.pow(y - x + rx, 3) / denominator;
            double frontRightPower = Math.pow(y - x - rx, 3) / denominator;
            double backRightPower = Math.pow(y + x - rx,3) / denominator;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            return TRUE;
        }
    }


    public Action drivetrain() {
        return new drivetrain();
    }
}