package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.IntakeLauncher;
import org.firstinspires.ftc.teamcode.utilities.BayesianOptimizer;

import java.util.Locale;


// TODO: The code appears to be correct, but its not great and manual tuning is much faster. See if theres a better approach to the problem. It likely lies in my loss function, rather than bayesian.
@Configurable
@TeleOp(name = "PID Optimizer", group = "Test")
@Disabled
// WARNING: THIS DOESNT WORK ANYMORE
public class PIDOptimizer extends OpMode {

    // Bounds for Bayesian Optimization - VERY CONSERVATIVE FOR SAFETY
    public static double MIN_P = 0.0;
    public static double MAX_P = 0.1; // Reduced from 0.1
    public static double MIN_I = 0.0;
    public static double MAX_I = 0.001; // Reduced from 0.001
    public static double MIN_D = 0.0;
    public static double MAX_D = 0.005; // Reduced from 0.005
    public static double MIN_F = 0.0;
    public static double MAX_F = 0.1; // Reduced from 0.1

    public static double LARGE_TARGET = 90;
    public static double SMALL_TARGET = 1;
    public static int TRIALS_PER_CONFIG = 1; // Reduced to 1 for speed
    public static double MAX_STAGE_DURATION = 2.0; // Max time per stage
    public static double SETTLE_DURATION = 0.5; // Time to hold position to exit early - increased to reduce jitter
    public static double DEADBAND_TOLERANCE = 0.5; // Degrees - stop moving when within this range

    private IntakeLauncher intakeLauncher;
    private BayesianOptimizer optimizer;
    private final ElapsedTime testTimer = new ElapsedTime();
    private double oscillationPenalty = 0;
    private double totalSettlingTime = 0;
    private double totalFinalError = 0;
    private double totalOvershoot = 0;
    private double maxOvershoot = 0;
    private double lastUnsettledTime = 0;
    private double lastError = 0;
    private int samples = 0;
    private boolean isTesting = false;
    private boolean hasEverSettled = false;
    private boolean isCurrentlySettled = false;
    private double settledStartTime = 0;
    private TelemetryManager telemetryM;
    private double currentTarget = 0;
    private int testStage = 0; // 0: Large Out, 1: Large Back, 2: Small Out, 3: Small Back
    private double startAngle = 0;
    private int currentTrial = 0;
    private double[] trialErrors;
    private double lastAveragedError = 0;

    private double totalJerk = 0;
    private double lastDerivative = 0;
    private double lastTime = 0;

    private boolean emergencyStop = false;
    private boolean skipCurrentConfig = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intakeLauncher = new IntakeLauncher(hardwareMap, telemetry);


        // Define bounds for P, I, D, F
        double[][] bounds = {
            {MIN_P, MAX_P},
            {MIN_I, MAX_I},
            {MIN_D, MAX_D},
            {MIN_F, MAX_F}
        };

        optimizer = new BayesianOptimizer(
                bounds,
                this::applyParams,
                this::getError
        );

        // Seed with initial PIDF values
        double[] currentParams = {0, 0, 0, 0};
        optimizer.setInitialGuess(currentParams);

        trialErrors = new double[TRIALS_PER_CONFIG];

        telemetryM.addData("Status", "Initialized. Press Play to start optimization.");
        telemetryM.update();
    }

    @Override
    public void loop() {
        // SAFETY: Emergency stop button
        if (gamepad1.y) {
            emergencyStop = true;
            telemetryM.addData("⚠️ EMERGENCY STOP", "Pressed Y - Stopping optimizer");
            requestOpModeStop();
            return;
        }

        // SAFETY: Skip current bad config
        if (gamepad1.x) {
            skipCurrentConfig = true;
            telemetryM.addData("⚠️ SKIP", "Pressed X - Skipping bad config");
        }

        if (!isTesting) {
            // Start a new test cycle
            startTest();
        } else {
            // Run test
            intakeLauncher.updateTurret(new Pose());

            // Accumulate error (Integral of Absolute Error)
            double currentAngle = intakeLauncher.getCurrentTurnAngle();
            double currentError = currentAngle - currentTarget;
            double absError = Math.abs(currentError);

            // SAFETY: Manual skip
            if (skipCurrentConfig) {
                telemetryM.addData("⚠️ SKIPPED", "Config aborted");
                totalFinalError += 5000;
                testStage = 4;
                skipCurrentConfig = false;
            }

            // Apply tolerance (use deadband)
            if (absError < DEADBAND_TOLERANCE) {
                absError = 0;
                hasEverSettled = true;
                if (!isCurrentlySettled) {
                    isCurrentlySettled = true;
                    settledStartTime = testTimer.seconds();
                }
            } else {
                isCurrentlySettled = false;
                lastUnsettledTime = testTimer.seconds();
            }

            // Track overshoot
            if (!intakeLauncher.shouldTurnLeft(currentAngle, currentTarget)) {
                maxOvershoot = Math.max(maxOvershoot, currentAngle - currentTarget);
            } else {
                maxOvershoot = Math.max(maxOvershoot, currentTarget - currentAngle);
            }

            // Penalize oscillations (crossing the setpoint)
            // Only count if we have settled at least once (to avoid counting initial crossing)
            // OR if we are unstable (crossing repeatedly)
            if (hasEverSettled && Math.signum(currentError) != Math.signum(lastError) && absError > 0.5) {
                oscillationPenalty+=1;
                oscillationPenalty *= 1.5;
            }
            // Calculate Jerk (change in derivative) to detect jitter
            double currentTime = testTimer.seconds();
            double dt = currentTime - lastTime;
            if (dt > 0) {
                double derivative = (currentError - lastError) / dt;
                double jerk = Math.abs(derivative - lastDerivative);
                totalJerk += jerk * dt; // Integral of absolute jerk
                lastDerivative = derivative;
                lastTime = currentTime;
            }

            lastError = currentError;

            samples++;

            // Safety check: if system is way off target, this config is bad
            if (absError > 90 || oscillationPenalty >= 5) {
                telemetryM.addData("⚠️ WARNING", "System unstable, skipping config");
                // Assign very high penalty and end test early
                totalFinalError += 1000;
                oscillationPenalty += 100;
                testStage = 4; // Force end of all stages
            }

            telemetryM.addData("Target Angle", currentTarget);
            telemetryM.addData("Current Angle", currentAngle);
            telemetryM.addData("Error", String.format(Locale.US, "%.2f", currentError));
            telemetryM.addData("Abs Error", String.format(Locale.US, "%.2f", absError));
            telemetryM.addData("Test Stage", testStage);
            telemetryM.addData("Trial", currentTrial + 1);
            telemetryM.addData("", "");
            telemetryM.addData("🛑 PRESS Y", "Emergency Stop");
            telemetryM.addData("⏭️ PRESS X", "Skip Bad Config");
            telemetryM.addData("", "");
            telemetryM.addData("IMU Working?", intakeLauncher.getCurrentTurnAngle() != 0 ? "YES" : "CHECK");

            // Check for stage completion (Time limit OR Early Exit)
            boolean stageComplete = false;
            if (isCurrentlySettled && (testTimer.seconds() - settledStartTime > SETTLE_DURATION)) {
                stageComplete = true; // Early exit!
            }
            if (testTimer.seconds() > MAX_STAGE_DURATION) {
                stageComplete = true; // Timeout
            }

            if (stageComplete) {
                // Accumulate metrics for this stage
                // Normalize error by target size to balance large/small movements
                double targetSize = Math.abs(currentTarget - startAngle);
                if (targetSize < 0.1) targetSize = 1.0; // Avoid division by zero

                // Add normalized settling time (ratio of max duration)
                // If we exited early, lastUnsettledTime is small.
                totalSettlingTime += lastUnsettledTime / MAX_STAGE_DURATION;

                // Add normalized final error
                if (absError > 0.5) totalFinalError += absError / targetSize;
                else totalFinalError = 0;

                // Add normalized overshoot
                totalOvershoot += maxOvershoot / targetSize;

                testStage++;
                testTimer.reset();
                lastUnsettledTime = 0; // Reset for next stage
                maxOvershoot = 0; // Reset for next stage
                hasEverSettled = false; // Reset for next stage
                isCurrentlySettled = false;
                startAngle = currentAngle; // Update start angle for next move

                if (testStage == 1) {
                    // Large Back
                    currentTarget = LARGE_TARGET;
                    intakeLauncher.setTargetTurnAngle(currentTarget);
                } else if (testStage == 2) {
                    // Small Out
                    currentTarget = 0;
                    intakeLauncher.setTargetTurnAngle(currentTarget);
                } else if (testStage == 3) {
                    // Small Back
                    currentTarget = Math.random() >= 0.5 ? 1 : -1 * SMALL_TARGET;
                    intakeLauncher.setTargetTurnAngle(currentTarget);
                } else if (testStage > 3) {
                    // Finish test and store error for this trial
                    double trialError = calculateCurrentError();
                    trialErrors[currentTrial] = trialError;
                    currentTrial++;

                    if (currentTrial < TRIALS_PER_CONFIG) {
                        // Run another trial with same parameters
                        isTesting = false; // Will restart in next loop
                    } else {
                        // All trials complete, average and update optimizer
                        double avgError = 0;
                        for (double err : trialErrors) {
                            avgError += err;
                        }
                        avgError /= TRIALS_PER_CONFIG;

                        // Store averaged error for optimizer to use
                        lastAveragedError = avgError;

                        boolean finished = optimizer.update();
                        isTesting = false;
                        currentTrial = 0; // Reset for next configuration

                        if (finished) {
                            telemetryM.addData("Status", "Optimization Finished!");
                            double[] best = optimizer.getBestParams();
                            telemetryM.addData("Best P", best[0]);
                            telemetryM.addData("Best I", best[1]);
                            telemetryM.addData("Best D", best[2]);
                            telemetryM.addData("Best F", best[3]);
                            requestOpModeStop();
                        }
                    }
                }

                // Update startAngle for next stage overshoot calc
                startAngle = intakeLauncher.getCurrentTurnAngle();
            }
        }

        telemetryM.addData("Iteration", optimizer.getIteration());
        telemetryM.addData("Samples", optimizer.getSampleCount());
        telemetryM.addData("Best Error", String.format(Locale.US, "%.2f", optimizer.getBestError()));

        // Always show best parameters found so far
        double[] bestParams = optimizer.getBestParams();
        telemetryM.addData("Best P", String.format(Locale.US, "%.6f", bestParams[0]));
        telemetryM.addData("Best I", String.format(Locale.US, "%.6f", bestParams[1]));
        telemetryM.addData("Best D", String.format(Locale.US, "%.6f", bestParams[2]));
        telemetryM.addData("Best F", String.format(Locale.US, "%.6f", bestParams[3]));
        telemetryM.addData("", "--- Current Test ---");

        if (currentTrial > 0 || lastAveragedError > 0) {
            telemetryM.addData("Last Avg Error", String.format(Locale.US, "%.2f", lastAveragedError));
        }
        telemetryM.update();
    }

    private void startTest() {
        isTesting = true;
        testTimer.reset();
        oscillationPenalty = 0;
        totalSettlingTime = 0;
        totalFinalError = 0;
        totalOvershoot = 0;
        maxOvershoot = 0;
        lastUnsettledTime = 0;
        lastError = 0;
        samples = 0;
        totalJerk = 0;
        lastDerivative = 0;
        lastTime = 0;
        testStage = 0;
        hasEverSettled = false;
        isCurrentlySettled = false;
    }

    private void applyParams(double[] params) {

    }

    private Double getError() {
        // Return the averaged error from all trials
        // This is called by BayesianOptimizer to get the error for the last parameter configuration
        return lastAveragedError;
    }

    private double calculateCurrentError() {
        // Calculate error for current test run
        // Weights:
        // Accuracy (Final Error): High (200)
        // Stability (Oscillations): High (50) - increased to heavily penalize oscillation
        // Speed (Settling Time): Low (10)
        // Overshoot: Medium (30)
        // Jitter (Total Jerk): Very High (10.0) - MASSIVELY increased to penalize jittery control

        double accuracyScore = totalFinalError * 100;

        double stabilityScore = oscillationPenalty * 50;

        double speedScore = totalSettlingTime * 100;

        double total = accuracyScore + stabilityScore + speedScore;
        telemetryM.addData("accuracy", accuracyScore/total);
        telemetryM.addData("stability", stabilityScore/total);
        telemetryM.addData("speed", speedScore/total);
        return total;
    }
}
