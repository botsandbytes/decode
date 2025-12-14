package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class BBPinPointDrive {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private DcMotorEx leftBack;


    // Constant

    public static double PROPORTIONAL_GAIN = 1; // Adjust this value to tune turning speed and stability
    public static double MIN_POWER = 0.5; // Minimum power to overcome friction
    public static double THRESHOLD_RADIANS = Math.toRadians(1);

    public double turnPower;

    // Flag so other files can tell if this is TeleOp
    public boolean isTeleOp = false;

    public BBPinPointDrive(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;
        initDrive(hardwareMap);
    }

    private void initDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        assert leftFront != null;
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        assert leftBack != null;
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        assert rightBack != null;
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        assert rightFront != null;
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


    public void turn(double angleDifference) {
        // Convert target angle to radians for internal calculations (most systems use radians)
        // Normalize the target angle to be between -PI and PI, if needed for your system
        // ... (example: see search result 1.5.5 for angle normalization logic)

        // Normalize the angle difference to the range [-PI, PI] to handle wrapping (e.g., turning from 170 deg to -170 deg)

         // Stop when within 2 degrees of the target

            // Calculate motor power using proportional control
            turnPower = angleDifference * PROPORTIONAL_GAIN;
            turnPower = Math.clamp(turnPower, 0, 1);

            // Apply a minimum power to prevent the robot from stalling near the target
            if (Math.abs(turnPower) < MIN_POWER) {
                turnPower = Math.copySign(MIN_POWER, turnPower);
            }

            // Set motor powers for differential turn
            double y = 0; // Remember, Y stick value is reversed
            double x = 0; // Counteract imperfect strafing
            double rx = turnPower;

            // Calculate motor powers
            double denominator = Math.max(Math.pow(Math.abs(y) + Math.abs(x) + Math.abs(rx), 3), 1);
            double frontLeftPower = Math.pow(y + x + rx, 3) / denominator;
            double backLeftPower = Math.pow(y - x + rx, 3) / denominator;
            double frontRightPower = Math.pow(y - x - rx, 3) / denominator;
            double backRightPower = Math.pow(y + x - rx,3) / denominator;

            // Set motor powers
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // Add a small sleep/pause if necessary to prevent the control loop from running too fast for your hardware

        // Stop the motors once the target is reached
        leftFront.setPower(0);
        rightFront.setPower(0);

}}