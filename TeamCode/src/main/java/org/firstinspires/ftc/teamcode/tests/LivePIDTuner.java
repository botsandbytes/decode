package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.IntakeLauncher;

/**
 * Live PIDF Tuner - Manual tuning with comprehensive feedback
 *
 * This is the RECOMMENDED approach for FTC PIDF tuning:
 * - Fast iteration (immediate feedback)
 * - Intuitive (see what each parameter does)
 * - Educational (learn control theory)
 * - Practical (tune in 2-5 minutes vs 10+ for auto-optimizer)
 *
 * CONTROLS:
 * - D-Pad Up/Down: Cycle through target angles
 * - A: Set small target (1°)
 * - B: Set medium target (45°)
 * - Y: Set large target (90°)
 * - X: Return to zero
 * - Left Bumper: Quick reset PID state
 *
 * TUNING GUIDE:
 * 1. Set all to zero except F (feedforward for velocity)
 * 2. Increase P until small oscillations appear
 * 3. Add D to dampen oscillations
 * 4. Add I only if steady-state error persists (usually not needed)
 * 5. Fine-tune all parameters
 *
 * Use FTC Dashboard or configurables to adjust values live!
 */
@Configurable
@TeleOp(name = "Live PID Tuner", group = "Test")
@Disabled
public class LivePIDTuner extends OpMode {

    // Make these configurable via FTC Dashboard or Configurables
    public static double P = 0.01;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.04;

    public static double TARGET_ANGLE = 90.0;
    public static double TOLERANCE = 0.5;

    private IntakeLauncher intakeLauncher;
    private TelemetryManager telemetryM;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime settlingTimer = new ElapsedTime();

    // Performance metrics
    private double maxOvershoot = 0;
    private double maxError = 0;
    private boolean hasSettled = false;
    private double settlingTime = 0;
    private int oscillationCount = 0;
    private double lastError = 0;

    // History tracking for visualization
    private static final int HISTORY_SIZE = 100;
    private double[] errorHistory = new double[HISTORY_SIZE];
    private int historyIndex = 0;

    private double startAngle = 0;
    private boolean testRunning = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intakeLauncher = new IntakeLauncher(hardwareMap, telemetry);

        telemetryM.addData("Status", "Initialized");
        telemetryM.addData("Instructions", "Use dashboard to adjust P, I, D, F");
        telemetryM.addData("", "Press A/B/Y/X to test different movements");
        telemetryM.update();
    }

    @Override
    public void loop() {
        // Apply current PIDF values

        // Handle controls
        handleControls();

        // Update controller
        intakeLauncher.updateTurret(0);

        // Collect metrics
        if (testRunning) {
            collectMetrics();
        }

        // Display comprehensive telemetry
        displayTelemetry();
    }

    private void handleControls() {
        // Start new test runs
        if (gamepad1.a && !testRunning) {
            startTest(1.0);
        } else if (gamepad1.b && !testRunning) {
            startTest(45.0);
        } else if (gamepad1.y && !testRunning) {
            startTest(90.0);
        } else if (gamepad1.x && !testRunning) {
            startTest(0.0);
        }

        // Reset PID state
        if (gamepad1.left_bumper) {
            // Reset integral and derivative tracking
            // This would require adding a reset method to IntakeLauncher
            testRunning = false;
            resetMetrics();
        }

        // Stop test
        if (gamepad1.right_bumper && testRunning) {
            testRunning = false;
        }
    }

    private void startTest(double targetOffset) {
        startAngle = intakeLauncher.getCurrentTurnAngle();
        TARGET_ANGLE = targetOffset;
        intakeLauncher.setTargetTurnAngle(TARGET_ANGLE);

        testRunning = true;
        runtime.reset();
        settlingTimer.reset();
        resetMetrics();
    }

    private void resetMetrics() {
        maxOvershoot = 0;
        maxError = 0;
        hasSettled = false;
        settlingTime = 0;
        oscillationCount = 0;
        lastError = 0;
        errorHistory = new double[HISTORY_SIZE];
        historyIndex = 0;
    }

    private void collectMetrics() {
        double currentAngle = intakeLauncher.getCurrentTurnAngle();
        double error = TARGET_ANGLE - currentAngle;
        double absError = Math.abs(error);

        // Track error history
        errorHistory[historyIndex % HISTORY_SIZE] = error;
        historyIndex++;

        // Track maximum error
        if (absError > maxError) {
            maxError = absError;
        }

        // Track overshoot (error changes sign and magnitude exceeds tolerance)
        if (Math.signum(error) != Math.signum(TARGET_ANGLE - startAngle)) {
            double overshoot = Math.abs(currentAngle - TARGET_ANGLE);
            if (overshoot > maxOvershoot) {
                maxOvershoot = overshoot;
            }
        }

        // Count oscillations (zero crossings with significant magnitude)
        if (Math.signum(error) != Math.signum(lastError) && absError > TOLERANCE) {
            oscillationCount++;
        }
        lastError = error;

        // Track settling time
        if (absError < TOLERANCE) {
            if (!hasSettled) {
                settlingTime = runtime.seconds();
                hasSettled = true;
            }
        } else {
            hasSettled = false;
        }

        // Auto-stop after 5 seconds
        if (runtime.seconds() > 5.0) {
            testRunning = false;
        }
    }

    private void displayTelemetry() {
        // Current state
        telemetryM.addData("=== CURRENT STATE ===", "");
        telemetryM.addData("Target Angle", TARGET_ANGLE);
        telemetryM.addData("Current Angle", intakeLauncher.getCurrentTurnAngle());
        telemetryM.addData("Error", TARGET_ANGLE - intakeLauncher.getCurrentTurnAngle());

        // PIDF values
        telemetryM.addData("=== PIDF VALUES ===", "");
        telemetryM.addData("P", P);
        telemetryM.addData("I", I);
        telemetryM.addData("D", D);
        telemetryM.addData("F", F);

        // Performance metrics
        if (testRunning || runtime.seconds() < 10) {
            telemetryM.addData("=== PERFORMANCE ===", "");
            telemetryM.addData("Test Time", runtime.seconds());
            telemetryM.addData("Max Error", maxError);
            telemetryM.addData("Max Overshoot", maxOvershoot);
            telemetryM.addData("Settling Time", hasSettled ? String.format("%.2fs", settlingTime) : "Not settled");
            telemetryM.addData("Oscillations", oscillationCount);

            // Performance assessment
            String assessment = assessPerformance();
            telemetryM.addData("Assessment", assessment);
        }

        // Controls
        telemetryM.addData("=== CONTROLS ===", "");
        telemetryM.addData("A", "Test 1° movement");
        telemetryM.addData("B", "Test 45° movement");
        telemetryM.addData("Y", "Test 90° movement");
        telemetryM.addData("X", "Return to 0°");
        telemetryM.addData("D-Pad", "Fine adjust target");
        telemetryM.addData("LB", "Reset PID state");
        telemetryM.addData("RB", "Stop test");

        // Error visualization (simple ASCII chart)
        if (testRunning && historyIndex > 10) {
            telemetryM.addData("=== ERROR PLOT ===", "");
            telemetryM.addData("", createSimplePlot());
        }

        telemetryM.update();
    }

    private String assessPerformance() {
        StringBuilder assessment = new StringBuilder();

        // Check settling time
        if (hasSettled && settlingTime < 0.5) {
            assessment.append("✓ Fast settling ");
        } else if (hasSettled && settlingTime < 1.5) {
            assessment.append("○ Moderate settling ");
        } else {
            assessment.append("✗ Slow settling ");
        }

        // Check overshoot
        double overshootPercent = (maxOvershoot / Math.abs(TARGET_ANGLE - startAngle)) * 100;
        if (overshootPercent < 5) {
            assessment.append("✓ No overshoot ");
        } else if (overshootPercent < 15) {
            assessment.append("○ Small overshoot ");
        } else {
            assessment.append("✗ Large overshoot ");
        }

        // Check oscillations
        if (oscillationCount <= 1) {
            assessment.append("✓ Stable");
        } else if (oscillationCount <= 3) {
            assessment.append("○ Some oscillation");
        } else {
            assessment.append("✗ Oscillatory");
        }

        // Provide tuning hints
        assessment.append("\n");
        if (oscillationCount > 3) {
            assessment.append("Hint: Decrease P or increase D");
        } else if (!hasSettled && runtime.seconds() > 3) {
            assessment.append("Hint: Increase P");
        } else if (overshootPercent > 15) {
            assessment.append("Hint: Increase D or decrease P");
        } else if (hasSettled && Math.abs(lastError) > TOLERANCE) {
            assessment.append("Hint: Add small I term");
        } else if (hasSettled && settlingTime > 1.5) {
            assessment.append("Hint: Increase P slightly");
        } else {
            assessment.append("Looking good! Try different movements.");
        }

        return assessment.toString();
    }

    private String createSimplePlot() {
        // Create a simple ASCII bar chart of recent error
        StringBuilder plot = new StringBuilder();
        int plotWidth = 20;
        int startIdx = Math.max(0, historyIndex - plotWidth);

        for (int i = startIdx; i < historyIndex; i++) {
            double error = errorHistory[i % HISTORY_SIZE];
            int barSize = (int) (Math.abs(error) / 2.0); // Scale factor
            barSize = Math.min(barSize, 20);

            plot.append(String.format("%4.1f° ", error));
            if (error > 0) {
                plot.append("+".repeat(barSize));
            } else {
                plot.append("-".repeat(barSize));
            }
            plot.append("\n");
        }

        return plot.toString();
    }
}

