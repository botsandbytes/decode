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

    // Configurable Parameters
    public static double INV_COEFF_A = 0.001;
    public static double INV_COEFF_B = 0.02;
    public static double INV_COEFF_C = 0.1;
    public static double TARGET_DISTANCE_R = 20.0;
    public static double LEFT_MOTOR_MULTIPLIER = 1.0;
    public static double RIGHT_MOTOR_MULTIPLIER = 1.0;
    public static double FEED_SERVO_POWER = 1.0;
    public static final double MAX_SHOOTER_VELOCITY = 2250.0;
    public static final double SHOOT_DURATION_SECONDS = 3.0;

    private static double calculatedPower = 0;

    private static boolean compControl;

    // Hardware
    private DcMotorEx shooterMotorLeft, shooterMotorRight;
    private CRServo feederServoLeft, feederServoRight, midServoLeft, midServoRight;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // Telemetry
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final GraphManager graphManager = PanelsGraph.INSTANCE.getManager();

    // State
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private double manualPower = 0.0;
    private boolean isManualControl = true, isShooting = false;
    private boolean shooterToggleActive = false, servoToggleActive = false, reverseServoToggleActive = true;
    private boolean aPrev = false, xPrev = false, yPrev = false, bPrev = false, lbPrev = false;

    private double calculateRequiredPower(double R) {
        double power = INV_COEFF_A * R * R + INV_COEFF_B * R + INV_COEFF_C;
        return Math.max(0.0, Math.min(1.0, power));
    }

    @Override
    public void init() {
        compControl = false;
        try {
            shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooter2");
            shooterMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

            feederServoLeft = hardwareMap.get(CRServo.class, "servo_left");
            feederServoRight = hardwareMap.get(CRServo.class, "servo_right");
            midServoLeft = hardwareMap.get(CRServo.class, "mid_left");
            midServoRight = hardwareMap.get(CRServo.class, "mid_right");
            feederServoLeft.setDirection(CRServo.Direction.REVERSE);
            midServoLeft.setDirection(CRServo.Direction.REVERSE);

            frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
            backLeftMotor = hardwareMap.dcMotor.get("leftBack");
            frontRightMotor = hardwareMap.dcMotor.get("rightFront");
            backRightMotor = hardwareMap.dcMotor.get("rightBack");
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Check motor/servo configuration names!");
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        // Button toggles
//        if (gamepad1.left_bumper && !lbPrev) {
//            isManualControl = !isManualControl;
//            if (isManualControl) manualPower = shooterMotorRight.getPower();
//        }
        if (gamepad1.a && !aPrev) servoToggleActive = !servoToggleActive;
        if (gamepad1.y && !yPrev) shooterToggleActive = !shooterToggleActive;
        if (gamepad1.b && !bPrev) reverseServoToggleActive = !reverseServoToggleActive;
        if (gamepad1.x && !xPrev && !isShooting) {
            isShooting = true;
            shooterTimer.reset();
        }

        // Update button states
        aPrev = gamepad1.a;
        xPrev = gamepad1.x;
        yPrev = gamepad1.y;
        bPrev = gamepad1.b;
        lbPrev = gamepad1.left_bumper;

        // Mecanum drive
             double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Calculate motor powers
            double denominator = Math.max(Math.pow(Math.abs(y) + Math.abs(x) + Math.abs(rx), 3), 1);

            // Set motor powers
            frontLeftMotor.setPower(Math.pow(y + x + rx, 3) / denominator);
            backLeftMotor.setPower(Math.pow(y - x + rx, 3) / denominator);
            frontRightMotor.setPower(Math.pow(y - x - rx, 3) / denominator);
            backRightMotor.setPower(Math.pow(y + x - rx,3) / denominator);

        // Calculate shooter power
        if (!compControl) {
            calculatedPower = isManualControl ?
                    Math.max(0.0, Math.min(1.0, manualPower -= gamepad1.right_stick_y * 0.02)) :
                    calculateRequiredPower(TARGET_DISTANCE_R);
        }
        // Check timed shooting
        boolean shootingActive = isShooting && shooterTimer.seconds() < SHOOT_DURATION_SECONDS;
        if (isShooting && !shootingActive) isShooting = false;

        // Apply shooter motor power
        boolean motorsShouldRun = shooterToggleActive || shootingActive;
        double finalPower = motorsShouldRun ? calculatedPower : 0.0;
        shooterMotorRight.setVelocity(0.9 * MAX_SHOOTER_VELOCITY * finalPower * RIGHT_MOTOR_MULTIPLIER);
        shooterMotorLeft.setVelocity(0.9 * MAX_SHOOTER_VELOCITY * finalPower * LEFT_MOTOR_MULTIPLIER);

        // Apply servo power
        double midServoPower = motorsShouldRun ? FEED_SERVO_POWER : 0.0;
        double lowerLeftServoPower =  (servoToggleActive || shootingActive) ? -FEED_SERVO_POWER : 0.0;
        double lowerRightServoPower = motorsShouldRun ? -FEED_SERVO_POWER : !reverseServoToggleActive ? FEED_SERVO_POWER: lowerLeftServoPower;
        feederServoLeft.setPower(lowerLeftServoPower);
        feederServoRight.setPower(lowerRightServoPower);
        midServoLeft.setPower(midServoPower);
        midServoRight.setPower(midServoPower);

        // Telemetry
        double targetVelocity = 0.9 * MAX_SHOOTER_VELOCITY * finalPower;
        telemetryM.addData("Mode (LB)", isManualControl ? "MANUAL" : "AUTOMATIC");
        telemetryM.addData("Shooting (X)", shootingActive ?
            String.format(java.util.Locale.US, "ACTIVE (%.1fs / 3.0s)", shooterTimer.seconds()) : "IDLE");
        telemetryM.addData("Shooter Toggle (Y)", shooterToggleActive ? "ON" : "OFF");
        telemetryM.addData("Lower Servos FWD (B)", servoToggleActive ? "ON" : "OFF");
        telemetryM.addData("Lower Servos REV (A)", reverseServoToggleActive ? "ON" : "OFF");
        telemetryM.addData("Target Distance (R)", TARGET_DISTANCE_R);
        telemetryM.addData("Base Power", calculatedPower);
        telemetryM.addData("Right Motor Power", shooterMotorRight.getVelocity());
        telemetryM.addData("Right Motor Target", targetVelocity * LEFT_MOTOR_MULTIPLIER);
        telemetryM.addData("Left Motor Power", shooterMotorLeft.getVelocity());
        telemetryM.addData("Left Motor Target", targetVelocity * LEFT_MOTOR_MULTIPLIER);
        telemetryM.addData("Lower Left Servos Power", lowerLeftServoPower);
        telemetryM.addData("Lower Right Servos Power", lowerRightServoPower);
        telemetryM.addData("Mid Servos Power", midServoPower);
        telemetryM.debug("MODEL PARAMS");
        telemetryM.debug("INV_A (R^2 Coeff) " + INV_COEFF_A);
        telemetryM.debug("LEFT_MOTOR_MULT " + LEFT_MOTOR_MULTIPLIER);
        telemetryM.debug("RIGHT_MOTOR_MULT " + RIGHT_MOTOR_MULTIPLIER);

        graphManager.addData("Right Motor Vel", shooterMotorRight.getVelocity());
        graphManager.addData("Left Motor Vel", shooterMotorLeft.getVelocity());
        graphManager.update();
        telemetryM.update(telemetry);
    }
}
