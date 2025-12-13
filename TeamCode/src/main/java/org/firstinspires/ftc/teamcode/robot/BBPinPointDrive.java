package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BBPinPointDrive {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private DcMotorEx leftBack;

    public GoBildaPinpointDriver odo;

    // Constants
    public static double PINPOINT_X_OFFSET_MM = -177;
    public static double PINPOINT_Y_OFFSET_MM = 40;

    // Flag so other files can tell if this is TeleOp
    public boolean isTeleOp = false;

    public BBPinPointDrive(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;
        initDrive(hardwareMap);
        initPinpoint(hardwareMap);
    }

    private void initDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Use power directly
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void initPinpoint(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public void turn(double targetAngleDegrees) {
        // Convert target angle to radians for internal calculations (most systems use radians)
        double targetAngleRadians = Math.toRadians(targetAngleDegrees);
        // Normalize the target angle to be between -PI and PI, if needed for your system
        // ... (example: see search result 1.5.5 for angle normalization logic)

        double currentHeading = odo.getHeading(AngleUnit.DEGREES); // Get current heading in radians
        double angleDifference = targetAngleRadians - currentHeading;

        // Normalize the angle difference to the range [-PI, PI] to handle wrapping (e.g., turning from 170 deg to -170 deg)
        while (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
        while (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

        final double THRESHOLD_RADIANS = Math.toRadians(2); // Stop when within 2 degrees of the target
        final double PROPORTIONAL_GAIN = 0.5; // Adjust this value to tune turning speed and stability
        final double MIN_POWER = 0.1; // Minimum power to overcome friction

        while (Math.abs(angleDifference) > THRESHOLD_RADIANS) {
            // Calculate motor power using proportional control
            double turnPower = angleDifference * PROPORTIONAL_GAIN;

            // Apply a minimum power to prevent the robot from stalling near the target
            if (Math.abs(turnPower) < MIN_POWER) {
                turnPower = Math.copySign(MIN_POWER, turnPower);
            }

            // Set motor powers for differential turn
            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightBack.setPower(-turnPower);// One wheel goes forward, the other backward
            rightFront.setPower(-turnPower); // One wheel goes forward, the other backward

            // Update odometry and angle difference in the loop
            odo.update(); // Important: must call update() on every loop iteration
            currentHeading = odo.getHeading(AngleUnit.DEGREES);
            angleDifference = targetAngleRadians - currentHeading;

            // Re-normalize angle difference within the loop
            while (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
            while (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

            // Add a small sleep/pause if necessary to prevent the control loop from running too fast for your hardware
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Stop the motors once the target is reached
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
}