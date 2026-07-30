package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.records.PIDGains;

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
  protected double lastCrossingTime = -1.0;

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
    lastError = targetValue - currentVal;
    lastPeakError = 0;
    lastCrossingTime = -1.0;
  }

  public void cancel() {
    state = AutotuneState.IDLE;
    crossingTimes.clear();
    peakErrors.clear();
  }

  public double updateAutotune(double currentVal) {
    if (state != AutotuneState.OSCILLATING) {
      return 0;
    }

    double error = targetValue - currentVal;
    double currentTime = timer.seconds();

    // Detect zero crossings (require at least 120ms between distinct crossings to filter sensor
    // noise)
    if (lastError * error < 0
        && (lastCrossingTime < 0 || (currentTime - lastCrossingTime) >= 0.12)) {
      crossingTimes.add(currentTime);
      lastCrossingTime = currentTime;

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

    // Timeout after 25 seconds
    if (currentTime > 25.0) {
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

    if (periods.isEmpty() || peakErrors.size() <= 1) {
      state = AutotuneState.FAILED;
      return;
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

    if (avgAmplitude <= 1e-6 || calculatedPu <= 1e-6) {
      state = AutotuneState.FAILED;
      return;
    }

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

  public boolean isComplete() {
    return state == AutotuneState.COMPLETE;
  }

  public boolean isFailed() {
    return state == AutotuneState.FAILED;
  }

  public boolean isRunning() {
    return state == AutotuneState.OSCILLATING;
  }

  public PIDGains getZieglerNichols() {
    return new PIDGains(calculatedKp, calculatedKi, calculatedKd);
  }

  public PIDGains getTyreusLuyben() {
    return new PIDGains(calculatedKp_TL, calculatedKi_TL, calculatedKd_TL);
  }

  public PIDGains getPessen() {
    return new PIDGains(calculatedKp_Pessen, calculatedKi_Pessen, calculatedKd_Pessen);
  }

  public double getKu() {
    return calculatedKu;
  }

  public double getPu() {
    return calculatedPu;
  }

  public int getCrossingCount() {
    return crossingTimes.size();
  }
}
