package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
@TeleOp(name = "Manual Turret Turn Tester", group = "Test")
// Comment this out to add to the OpMode list
public class manualTurretTurnTest extends LinearOpMode
{
    // The IMU sensor object
    IMU imu;
    CRServo turn;
    private TelemetryManager telemetryM;
    public static int turn_degrees = 60;
    public static int error_tolerance = 2;
    public static double maxPower = 0.5, minPower = 0.13;
    public static  double Kp = 0.006;


    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    public void turnUsingPIDF() {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (turn_degrees > 115) {
            turn_degrees = 115;
        } else if (turn_degrees < -115) {
            turn_degrees = -115;
        }
        double error = turn_degrees - currentAngle;
        turn.setDirection(CRServo.Direction.FORWARD);
        while (Math.abs(error) > 1) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turn_degrees - currentAngle;
            double power = error * Kp;
            if (power > maxPower) {
                power = maxPower;
            } else if (power < minPower && power > 0) {
                power = minPower;
            } else if (power > -minPower && power < 0) {
                power = -minPower;
            } else if (power < -maxPower) {
                power = -maxPower;
            }
            telemetry.addData("Current Power", power);
            telemetry.addData("Error: ", error);
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
            turn.setPower(power);
        }
        turn.setPower(0);
    }

    @Override public void runOpMode() throws InterruptedException {

        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "turnImu");
        turn = hardwareMap.get(CRServo.class, "turn");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Loop and update the dashboard
        while (!isStopRequested()) {
            // Check to see if heading reset is requested
            if (gamepad1.dpad_down) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else  if (gamepad1.dpad_right) {
                turnUsingPIDF();
            } else {
                telemetry.addData("Yaw", "Press DPAD DOWN on Gamepad to reset\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }
}

