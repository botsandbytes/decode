package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.IntakeLauncher;

@Configurable
@TeleOp(name = "Live Shooter Tuner", group = "Test")
public class LiveShooterTuner extends OpMode {

    // =========================================================
    // TUNING VARIABLES (Edit live in Dashboard)
    // =========================================================
    public static double p = 0.001;
    public static double i = 0.0;
    public static double d = 0.0001;
    public static double f = 0.0; // Minimum power clamp (stiction)

    public static double TARGET_VELOCITY = 0;

    // =========================================================
    // SEQUENCE SETTINGS
    // =========================================================
    public static boolean runTestSequence = false;
    public static double MAX_STEP_DURATION = 2.0;
    public static double STABILITY_TOLERANCE = 50.0;
    public static double REQUIRED_STABLE_TIME = 0.2;

    // Magic Numbers
    private static final double MAX_RPM = 1500;
    private static final double TRAIN_RPM_PERCENT = 1.0;

    // =========================================================
    // STATE
    // =========================================================
    private IntakeLauncher intakeLauncher;
    private TelemetryManager telemetryM;
    private SimplePIDController pidController; // Custom Internal Controller

    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime stabilityTimer = new ElapsedTime();

    private int sequenceStep = 0;
    private boolean isStable = false;

    private double lastStepTimeTaken = 0;
    private String lastStepResult = "N/A";

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intakeLauncher = new IntakeLauncher(hardwareMap, telemetry);

        // Initialize our custom controller
        pidController = new SimplePIDController(p, i, d);

        intakeLauncher.stopLauncher();

        telemetryM.addData("Status", "Initialized");
        telemetryM.update();
    }

    @Override
    public void loop() {
        // Update coefficients live
        pidController.setPID(p, i, d);

        double currentVelocity = intakeLauncher.getShooterVelocity();
        double currentError = TARGET_VELOCITY - currentVelocity;

        // =========================================================
        // AUTOMATED SEQUENCE LOGIC
        // =========================================================
        if (gamepad1.a && !runTestSequence) {
            startSequence();
        }

        if (runTestSequence) {
            boolean currentlyInThreshold = Math.abs(currentError) < STABILITY_TOLERANCE;

            if (!currentlyInThreshold) {
                stabilityTimer.reset();
                isStable = false;
            } else if (stabilityTimer.seconds() > REQUIRED_STABLE_TIME) {
                isStable = true;
            }

            if (stepTimer.seconds() > MAX_STEP_DURATION || isStable) {
                advanceSequence();
            }
        }

        // =========================================================
        // PID LOOP
        // =========================================================

        // Calculate raw PID output using our custom class
        double pidPower = pidController.calculate(TARGET_VELOCITY, currentVelocity);
        double finalPower = 0;

        // Apply F as Stiction Clamp
        if (Math.abs(pidPower) > 1e-6) {
            double absPower = Math.abs(pidPower);
            // Java 21 Clamp
            absPower = Math.clamp(absPower, f, 1.0);
            finalPower = Math.copySign(absPower, pidPower);
        }

        intakeLauncher.runShooterRaw(finalPower);

        // =========================================================
        // TELEMETRY
        // =========================================================
        telemetryM.addData("Target Velocity", TARGET_VELOCITY);
        telemetryM.addData("Actual Velocity", currentVelocity);
        telemetryM.addData("PID Output", pidPower);

        if (runTestSequence) {
            telemetryM.addData("--- SEQUENCE STATUS ---", "");
            telemetryM.addData("Step", sequenceStep + "/5");
            telemetryM.addData("Time in Step", String.format("%.2fs", stepTimer.seconds()));
            telemetryM.addData("Last Move Time", String.format("%.2fs (%s)", lastStepTimeTaken, lastStepResult));
        }

        telemetryM.update();
    }

    private void startSequence() {
        runTestSequence = true;
        sequenceStep = 1;
        setTargetForStep(1);
        stepTimer.reset();
        stabilityTimer.reset();
        isStable = false;
    }

    private void advanceSequence() {
        lastStepTimeTaken = stepTimer.seconds();
        lastStepResult = isStable ? "Settled" : "Timed Out";
        sequenceStep++;

        if (sequenceStep > 5) {
            runTestSequence = false;
            sequenceStep = 0;
            TARGET_VELOCITY = calculateTargetVelocity(0.0);
        } else {
            setTargetForStep(sequenceStep);
            stepTimer.reset();
            stabilityTimer.reset();
            isStable = false;
        }
    }

    private void setTargetForStep(int step) {
        switch (step) {
            case 1: TARGET_VELOCITY = calculateTargetVelocity(0.60); break;
            case 2: TARGET_VELOCITY = calculateTargetVelocity(0.605); break;
            case 3: TARGET_VELOCITY = calculateTargetVelocity(0.90); break;
            case 4: TARGET_VELOCITY = calculateTargetVelocity(0.895); break;
            case 5: TARGET_VELOCITY = calculateTargetVelocity(0.0); break;
        }
    }

    private double calculateTargetVelocity(double percent) {
        return MAX_RPM * percent * TRAIN_RPM_PERCENT;
    }

    // =========================================================
    // CUSTOM PID CONTROLLER
    // =========================================================
    private static class SimplePIDController {
        private double kP, kI, kD;
        private double integralSum = 0;
        private double lastError = 0;
        private final ElapsedTime timer = new ElapsedTime();

        public SimplePIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            timer.reset();
        }

        public void setPID(double p, double i, double d) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
        }

        public double calculate(double target, double current) {
            // Calculate dt (delta time)
            double dt = timer.seconds();
            timer.reset();

            // Prevent divide by zero or huge derivative spikes on first loop
            if (dt < 1e-6) dt = 1e-6;

            double error = target - current;

            // Proportional
            double pOut = kP * error;

            // Integral
            integralSum += error * dt;
            double iOut = kI * integralSum;

            // Derivative
            double derivative = (error - lastError) / dt;
            double dOut = kD * derivative;

            lastError = error;

            return pOut + iOut + dOut;
        }
    }
}