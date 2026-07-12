package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

public class PIDAutotuner {

    public enum AutotuneState {
        IDLE,
        OSCILLATING,
        COMPLETE,
        FAILED
    }

    protected AutotuneState state = AutotuneState.IDLE;
    protected final ArrayList<Double> crossingTimes = new ArrayList<>();
    protected final ArrayList<Double> peakErrors = new ArrayList<>();
    protected final ElapsedTime timer = new ElapsedTime();

    protected double lastError = 0;
    protected double relayOutputMagnitude = 0.25;
    protected double targetValue = 0;
    protected double lastPeakError = 0;

    // Results
    protected double calculatedKu = 0;
    protected double calculatedPu = 0;

    // Ziegler-Nichols (aggressive, fast response)
    protected double calculatedKp = 0;
    protected double calculatedKi = 0;
    protected double calculatedKd = 0;

    // Tyreus-Luyben (conservative, no overshoot)
    protected double calculatedKp_TL = 0;
    protected double calculatedKi_TL = 0;
    protected double calculatedKd_TL = 0;

    // Pessen Integration Rule (recommended for high load holding)
    protected double calculatedKp_Pessen = 0;
    protected double calculatedKi_Pessen = 0;
    protected double calculatedKd_Pessen = 0;

    public void startAutotune(double currentVal, double targetVal, double outputMagnitude) {
        state = AutotuneState.OSCILLATING;
        targetValue = targetVal;
        relayOutputMagnitude = Math.clamp(outputMagnitude, 0.01, 1.0);

        crossingTimes.clear();
        peakErrors.clear();
        timer.reset();
        lastError = 0;
        lastPeakError = 0;
    }

    public double updateAutotune(double currentVal) {
        if (state != AutotuneState.OSCILLATING) {
            return 0;
        }

        double error = targetValue - currentVal;

        // Detect zero crossings (50ms debounce)
        if (lastError * error < 0 && timer.seconds() > 0.05) {
            crossingTimes.add(timer.seconds());

            if (Math.abs(lastError) > Math.abs(lastPeakError)) {
                lastPeakError = lastError;
            }
            peakErrors.add(Math.abs(lastPeakError));
            lastPeakError = 0;

            // Need 6 crossings for 3 full oscillations
            if (crossingTimes.size() >= 6) {
                calculatePIDGains();
                state = AutotuneState.COMPLETE;
                return 0;
            }
        }

        // Track peaks
        if (Math.abs(error) > Math.abs(lastPeakError)) {
            lastPeakError = error;
        }

        // Timeout after 15 seconds
        if (timer.seconds() > 15) {
            state = AutotuneState.FAILED;
            return 0;
        }

        lastError = error;

        // Return signed power
        return error > 0 ? relayOutputMagnitude : -relayOutputMagnitude;
    }

    protected void calculatePIDGains() {
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
        for (int i = 1; i < peakErrors.size(); i++) {
            avgAmplitude += peakErrors.get(i);
        }
        avgAmplitude /= (peakErrors.size() - 1);

        // Ultimate gain: Ku = (4 * d) / (π * a)
        calculatedKu = (4.0 * relayOutputMagnitude) / (Math.PI * avgAmplitude);

        // 1. Ziegler-Nichols (classic PID)
        calculatedKp = 0.6 * calculatedKu;
        calculatedKi = 2.0 * calculatedKp / calculatedPu;
        calculatedKd = calculatedKp * calculatedPu / 8.0;

        // 2. Tyreus-Luyben (conservative, NO overshoot)
        calculatedKp_TL = 0.45 * calculatedKu;
        calculatedKi_TL = calculatedKp_TL / (calculatedPu * 2.2);
        calculatedKd_TL = calculatedKp_TL * calculatedPu / 6.3;

        // 3. Pessen Integration (minimized integrated error)
        calculatedKp_Pessen = 0.7 * calculatedKu;
        calculatedKi_Pessen = 2.5 * calculatedKp_Pessen / calculatedPu;
        calculatedKd_Pessen = 0.15 * calculatedKp_Pessen * calculatedPu;
    }

    public boolean isComplete() { return state == AutotuneState.COMPLETE; }
    public boolean isFailed() { return state == AutotuneState.FAILED; }
    public boolean isRunning() { return state == AutotuneState.OSCILLATING; }

    public double getKp() { return calculatedKp; }
    public double getKi() { return calculatedKi; }
    public double getKd() { return calculatedKd; }
    public double getKu() { return calculatedKu; }
    public double getPu() { return calculatedPu; }
    public int getCrossingCount() { return crossingTimes.size(); }

    public double getKp_TL() { return calculatedKp_TL; }
    public double getKi_TL() { return calculatedKi_TL; }
    public double getKd_TL() { return calculatedKd_TL; }

    public double getKp_Pessen() { return calculatedKp_Pessen; }
    public double getKi_Pessen() { return calculatedKi_Pessen; }
    public double getKd_Pessen() { return calculatedKd_Pessen; }
}
