package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class TurretAutotuner {

    private enum AutotuneState {
        IDLE,
        OSCILLATING,
        COMPLETE,
        FAILED
    }

    private AutotuneState state = AutotuneState.IDLE;
    private final ArrayList<Double> crossingTimes = new ArrayList<>();
    private final ArrayList<Double> peakAngles = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double relayOutputMagnitude = 0.25;
    private double targetAngle = 0;
    private double lastPeakAngle = 0;

    // Results
    private double calculatedKu = 0;
    private double calculatedPu = 0;

    // Ziegler-Nichols (aggressive, fast response)
    private double calculatedKp = 0;
    private double calculatedKi = 0;
    private double calculatedKd = 0;

    // Tyreus-Luyben (conservative, no overshoot - RECOMMENDED for turrets)
    private double calculatedKp_TL = 0;
    private double calculatedKi_TL = 0;
    private double calculatedKd_TL = 0;

    public void startAutotune(double currentAngle, double outputMagnitude) {
        state = AutotuneState.OSCILLATING;
        targetAngle = currentAngle;
        relayOutputMagnitude = Math.clamp(outputMagnitude, 0.15, 0.5);

        crossingTimes.clear();
        peakAngles.clear();
        timer.reset();
        lastError = 0;
        lastPeakAngle = 0;
    }

    /**
     * Update autotune. Returns power to apply to servo.
     * Call this every loop while isRunning() returns true.
     */
    public double updateAutotune(double currentAngle) {
        if (state != AutotuneState.OSCILLATING) {
            return 0;
        }

        double error = targetAngle - currentAngle;

        // Detect zero crossings (FIXED: 50ms debounce instead of 200ms)
        if (lastError * error < 0 && timer.seconds() > 0.05) {
            crossingTimes.add(timer.seconds());

            if (Math.abs(lastError) > Math.abs(lastPeakAngle)) {
                lastPeakAngle = lastError;
            }
            peakAngles.add(Math.abs(lastPeakAngle));
            lastPeakAngle = 0;

            // Need 6 crossings for 3 full oscillations
            if (crossingTimes.size() >= 6) {
                calculatePIDGains();
                state = AutotuneState.COMPLETE;
                return 0;
            }
        }

        // Track peaks
        if (Math.abs(error) > Math.abs(lastPeakAngle)) {
            lastPeakAngle = error;
        }

        // Timeout after 15 seconds
        if (timer.seconds() > 15) {
            state = AutotuneState.FAILED;
            return 0;
        }

        lastError = error;

        // Return signed power (positive = forward direction)
        return error > 0 ? relayOutputMagnitude : -relayOutputMagnitude;
    }

    private void calculatePIDGains() {
        // Calculate period
        ArrayList<Double> periods = new ArrayList<>();
        for (int i = 2; i < crossingTimes.size(); i += 2) {
            double period = crossingTimes.get(i) - crossingTimes.get(i - 2);
            periods.add(period);
        }

        calculatedPu = 0;
        for (double p : periods) calculatedPu += p;
        calculatedPu /= periods.size();

        // Calculate amplitude
        double avgAmplitude = 0;
        for (int i = 1; i < peakAngles.size(); i++) {
            avgAmplitude += peakAngles.get(i);
        }
        avgAmplitude /= (peakAngles.size() - 1);

        // Ultimate gain: Ku = (4 * d) / (π * a)
        calculatedKu = (4.0 * relayOutputMagnitude) / (Math.PI * avgAmplitude);

        // 1. Ziegler-Nichols (aggressive, allows ~25% overshoot)
        calculatedKp = 0.6 * calculatedKu;
        calculatedKi = 2.0 * calculatedKp / calculatedPu;
        calculatedKd = calculatedKp * calculatedPu / 8.0;

        // 2. Tyreus-Luyben (conservative, NO overshoot - RECOMMENDED for turrets)
        calculatedKp_TL = 0.45 * calculatedKu;
        calculatedKi_TL = calculatedKp_TL / (calculatedPu * 2.2);
        calculatedKd_TL = calculatedKp_TL * calculatedPu / 6.3;
    }

    // State checks
    public boolean isComplete() { return state == AutotuneState.COMPLETE; }
    public boolean isFailed() { return state == AutotuneState.FAILED; }
    public boolean isRunning() { return state == AutotuneState.OSCILLATING; }

    // Getters
    public double getKp() { return calculatedKp; }
    public double getKi() { return calculatedKi; }
    public double getKd() { return calculatedKd; }
    public double getKu() { return calculatedKu; }
    public double getPu() { return calculatedPu; }
    public int getCrossingCount() { return crossingTimes.size(); }

    // Tyreus-Luyben getters (RECOMMENDED for turrets - no overshoot)
    public double getKp_TL() { return calculatedKp_TL; }
    public double getKi_TL() { return calculatedKi_TL; }
    public double getKd_TL() { return calculatedKd_TL; }
}

