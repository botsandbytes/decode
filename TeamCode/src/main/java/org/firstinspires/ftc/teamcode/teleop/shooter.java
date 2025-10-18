package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.graph.GraphManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Quadratic-reg turret")
@Configurable
public class shooter extends OpMode {

    // --- 1. Configurable Model Parameters (Tunable via the @Configurable System) ---

    // Model coefficients for the Inverse function: Power (P) = A*R^2 + B*R + C
    public static double INV_COEFF_A = 0.001;
    public static double INV_COEFF_B = 0.02;
    public static double INV_COEFF_C = 0.1;

    // The desired target distance (R)
    public static double TARGET_DISTANCE_R = 20.0;

    // Multiplier applied to the left motor power for differential spin control (Default 1.0 = equal power)
    public static double LEFT_MOTOR_MULTIPLIER = 1.0;

    // Power/Velocity for the CR feeder servos. Positive/Negative determines direction.
    public static double FEED_SERVO_POWER = 0.5;

    // Duration of the shot sequence after the trigger is pulled
    public static final double SHOOT_DURATION_SECONDS = 3.0;
    /*
     * FTC OpMode for real-time control of a dual-motor shooter with two continuous rotation (CR) feeder servos.
     * Shooter motors engage for a fixed duration when the X button is pressed.
     */

    // --- 2. Core Model Function ---

    /**
     * Calculates the base motor Power (P) from the Distance (R) using the configurable model.
     */
    private double calculateRequiredPower(double R) {
        final double a = INV_COEFF_A;
        final double b = INV_COEFF_B;
        final double c = INV_COEFF_C;

        // --- YOUR SIMPLE FORMULA GOES HERE (Currently P = A*R^2 + B*R + C) ---
        double power = a * Math.pow(R, 2) + b * R + c;

        // Clamp the result to the valid motor power range [0.0, 1.0]
        return Math.max(0.0, Math.min(1.0, power));
    }


    // --- 3. Hardware and Telemetry Setup ---

    private DcMotorEx shooterMotorLeft;
    private DcMotorEx shooterMotorRight;
    private CRServo feederServoLeft;
    private CRServo feederServoRight;
    private CRServo midServoLeft;
    private CRServo midServoRight;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    // Correct initialization using the custom static managers
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final GraphManager graphManager = PanelsGraph.INSTANCE.getManager();

    private final ElapsedTime shooterTimer = new ElapsedTime();

    private double manualPower = 0.0;
    private boolean isManualControl = false;
    private boolean isShooting = false;

    private boolean aButtonPrevState = false;
    private boolean xButtonPrevState = false;


    @Override
    public void init() {
        // Initialize dual shooter motors
        try {
            shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "left_front_drive");
            shooterMotorRight = hardwareMap.get(DcMotorEx.class, "right_front_drive");

            shooterMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

            // Initialize CR servos
            feederServoLeft = hardwareMap.get(CRServo.class, "servo_left");
            feederServoRight = hardwareMap.get(CRServo.class, "servo_right");
            midServoLeft = hardwareMap.get(CRServo.class, "mid_left");
            midServoRight = hardwareMap.get(CRServo.class, "mid_right");

            feederServoLeft.setDirection(CRServo.Direction.REVERSE);
            midServoLeft.setDirection(CRServo.Direction.REVERSE);

            // Motors start off
            shooterMotorLeft.setPower(0.0);
            shooterMotorRight.setPower(0.0);
        } catch (Exception e) {
            telemetry.addData("Error", "Check motor/servo configuration names!");
            telemetry.update();
        }

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // A Button handles mode toggle (Auto vs. Manual power source)
        boolean aButtonCurrentState = gamepad1.a;
        if (aButtonCurrentState && !aButtonPrevState) {
            isManualControl = !isManualControl;
            if (isManualControl) {
                // When switching to manual, start from the current right motor power
                manualPower = shooterMotorRight.getPower();
            }
        }
        aButtonPrevState = aButtonCurrentState;

        final double currentTargetR = TARGET_DISTANCE_R;

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


        // Determine the calculated power based on the current mode
        // --- State Variables ---
        double calculatedPower = 0.0;
        if (isManualControl) {
            // Manual Control: Use right joystick Y-axis for power adjustment
            manualPower -= gamepad1.right_stick_y * 0.02;
            manualPower = Math.max(0.0, Math.min(1.0, manualPower));
            calculatedPower = manualPower;
        } else {
            // Automatic Control: Use model output
            calculatedPower = calculateRequiredPower(currentTargetR);
        }

        // --- 3. Timed Shooting Logic (X Button) ---

        // X Button press starts the 2.0 second shot sequence
        boolean xButtonCurrentState = gamepad1.x;
        if (xButtonCurrentState && !xButtonPrevState) {
            if (!isShooting) {
                isShooting = true;
                shooterTimer.reset();
            }
        }
        xButtonPrevState = xButtonCurrentState;

        double finalPower = 0.0;

        if (isShooting) {
            if (shooterTimer.seconds() < SHOOT_DURATION_SECONDS) {
                // RUN motors: Apply the calculated power source
                // Turn on feeder servos during shooting
                feederServoLeft.setPower(FEED_SERVO_POWER);
                feederServoRight.setPower(FEED_SERVO_POWER);
                midServoLeft.setPower(FEED_SERVO_POWER);
                midServoRight.setPower(FEED_SERVO_POWER);
                finalPower = calculatedPower;
            } else {
                // STOP motors: Timer expired
                isShooting = false;
                // Turn off feeder servos
                feederServoLeft.setPower(0);
                feederServoRight.setPower(0);
                midServoLeft.setPower(0);
                midServoRight.setPower(0);
                finalPower = 0.0;
            }
        } else {
            // Ensure servos are off when not shooting
            feederServoLeft.setPower(0);
            feederServoRight.setPower(0);
            midServoLeft.setPower(0);
            midServoRight.setPower(0);
        }


        // Apply final power to motors
        shooterMotorRight.setPower(finalPower);
        shooterMotorLeft.setPower(finalPower * LEFT_MOTOR_MULTIPLIER);


        // --- 4. Telemetry and Graphing ---

        // Telemetry using the custom manager (for panel display)
        telemetryM.addData("Mode", isManualControl ? "MANUAL (Gamepad)" : "AUTOMATIC (Model)");
        telemetryM.addData("Shooting", isShooting ? "ACTIVE (" + String.format(java.util.Locale.US, "%.1f", shooterTimer.seconds()) + "s / 3.0s)" : "IDLE");
        telemetryM.addData("Target Distance (R)", currentTargetR);
        telemetryM.addData("Base Power Source", calculatedPower);

        telemetryM.addData("Right Motor Power", shooterMotorRight.getPower());
        telemetryM.addData("Left Motor Power", shooterMotorLeft.getPower());
        telemetryM.addData("Feeder Power (CR)", FEED_SERVO_POWER);

        // Telemetry for debugging the formula's inputs
        telemetryM.debug("MODEL PARAMS");
        telemetryM.debug("INV_A (R^2 Coeff) " + INV_COEFF_A);
        telemetryM.debug("LEFT_MOTOR_MULT " + LEFT_MOTOR_MULTIPLIER);

        // Graphing data updates
        graphManager.addData("Right Motor Power", shooterMotorRight.getPower());
        graphManager.addData("Left Motor Power", shooterMotorLeft.getPower());

        // Update both graph and telemetry views
        graphManager.update();
        telemetryM.update(telemetry); // Passes FTC telemetry object to update both displays
    }
}
