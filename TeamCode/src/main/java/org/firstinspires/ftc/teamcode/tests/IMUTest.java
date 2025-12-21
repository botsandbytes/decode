package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

@TeleOp(name = "IMU Test", group = "Test")
public class IMUTest extends OpMode {

    private IMU turnIMU;

    @Override
    public void init() {
        turnIMU = hardwareMap.get(IMU.class, "turnImu");

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        turnIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        turnIMU.resetYaw();

        telemetry.addData("Status", "Initialized - Rotate the turret manually");
        telemetry.update();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
        double yawRadians = orientation.getYaw(AngleUnit.RADIANS);
        double yawDegrees = Math.toDegrees(yawRadians);

        telemetry.addData("Yaw (radians)", String.format(Locale.US, "%.4f", yawRadians));
        telemetry.addData("Yaw (degrees)", String.format(Locale.US, "%.2f", yawDegrees));
        telemetry.addData("Pitch", String.format(Locale.US, "%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.addData("Roll", String.format(Locale.US, "%.2f", orientation.getRoll(AngleUnit.DEGREES)));
        telemetry.addData("", "");
        telemetry.addData("Instructions", "Manually rotate the turret");
        telemetry.addData("Expected", "Yaw should change as you rotate");
        telemetry.update();
    }
}

