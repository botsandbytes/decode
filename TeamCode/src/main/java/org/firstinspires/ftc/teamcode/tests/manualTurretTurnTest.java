package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
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
    public static double sleep_multiplier = 15.7;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.009, 0.0003, 0.003, 0.1); // (0.008, 0.0005, 0.003, 0.007);
    PIDFController pidfController = new PIDFController(pidfCoefficients);

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    public void turnUsingPIDF() {
        int error_tolerance = 2;
        double maxPower = 0.5, minPower = 0.15;
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        pidfController.updatePosition(currentAngle);
        pidfController.setTargetPosition(turn_degrees);
            turn.setDirection(CRServo.Direction.FORWARD);
            while (pidfController.getError() > error_tolerance || pidfController.getError() < -error_tolerance) {
                currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                pidfController.updatePosition(currentAngle);
                double power = pidfController.run();
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
                telemetry.addData("Error: ", pidfController.getError());
                telemetry.addData("Current Angle: ", currentAngle);
                telemetry.update();
                turn.setPower(power);
            }
            turn.setPower(0);
            pidfController.reset();
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
        pidfController.reset();

        // Loop and update the dashboard
        while (!isStopRequested()) {
            // Check to see if heading reset is requested
            if (gamepad1.dpad_down) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else  if (gamepad1.dpad_right) {
                pidfController.setTargetPosition(turn_degrees);
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

