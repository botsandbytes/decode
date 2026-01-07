package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
@TeleOp(name = "Manual Turn Tester", group = "Test")
// Comment this out to add to the OpMode list
public class manualTurretTurnTest extends LinearOpMode
{
    // The IMU sensor object
    IMU imu;
    CRServo turn;
    private TelemetryManager telemetryM;
    public static int turn_degrees = 60;
    public static double sleep_multiplier = 15.7;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.009, 0.0003, 0.003, 0.1); // (0.008, 0.0005, 0.003, 0.007);
    PIDFController pidfController = new PIDFController(pidfCoefficients);

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    public void turnUsingPIDF() {
        int error_tolerance = 2;
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        pidfController.updatePosition(currentAngle);
        pidfController.setTargetPosition(turn_degrees);
//        if (turn_degrees < 0) {
//            turn.setDirection(CRServo.Direction.FORWARD);
//            while (pidfController.getError() > 2 || pidfController.getError() < -2) {
//                currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//                pidfController.updatePosition(currentAngle);
//                double power = pidfController.run();
//                telemetry.addData("Current Power", power);
//                telemetry.addData("Error: ", pidfController.getError());
//                telemetry.addData("Current Angle: ", currentAngle);
//                telemetry.update();
//                turn.setPower(power);
//            }
//            turn.setPower(0);
//            pidfController.reset();
//        } else {
            turn.setDirection(CRServo.Direction.FORWARD);
            while (pidfController.getError() > 2 || pidfController.getError() < -2) {
                currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                pidfController.updatePosition(currentAngle);
                double power = pidfController.run();
                if (power > 0.5) {
                    power = 0.5;
                } else if (power < 0.15 && power > 0) {
                    power = 0.15;
                } else if (power > -0.15 && power < 0) {
                    power = -0.15;
                } else if (power < -0.5) {
                    power = -0.5;
                }
                telemetry.addData("Current Power", power);
                telemetry.addData("Error: ", pidfController.getError());
                telemetry.addData("Current Angle: ", currentAngle);
                telemetry.update();
                turn.setPower(power);
            }
            turn.setPower(0);
            pidfController.reset();
//        }

    }
    public void turnTurret(int degrees) {
        if (degrees > 0 ) {
            turn.setDirection(DcMotorSimple.Direction.REVERSE);
            turn.setPower(0.5);
            while(true) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double delta = abs(degrees - orientation.getYaw(AngleUnit.DEGREES));
                if (delta < 2) {
                    break;
                }
            }
            turn.setPower(0);
//            sleep((long) (degrees * sleep_multiplier));
        } else {
            turn.setDirection(DcMotorSimple.Direction.FORWARD);
            turn.setPower(0.5);
            while(true) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double delta = abs(degrees - orientation.getYaw(AngleUnit.DEGREES));
                if (delta < 2) {
                    break;
                }
            }
            turn.setPower(0);
//            sleep((long) (-degrees * sleep_multiplier));
        }
        turn.setPower(0);
    }

    @Override public void runOpMode() throws InterruptedException {

        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "turnImu");
        turn = hardwareMap.get(CRServo.class, "turn");


        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         *
         * If you are using a REV 9-Axis IMU, you can use the Rev9AxisImuOrientationOnRobot class instead of the
         * RevHubOrientationOnRobot class, which has an I2cPortFacingDirection instead of a UsbFacingDirection.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        pidfController.reset();

        // Loop and update the dashboard
        while (!isStopRequested()) {

            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            // Check to see if heading reset is requested
            if (gamepad1.dpad_down) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else  if (gamepad1.dpad_right) {
                pidfController.setTargetPosition(turn_degrees);
                turnUsingPIDF();
                //turnTurret(turn_degrees);
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

