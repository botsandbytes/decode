package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@TeleOp(name = "Turret Analog Calibration", group = "Tuning")
public class TurretAnalogCalibrationOpMode extends OpMode {

  private CRServo turnServo;
  private AnalogInput turnAnalog;
  private IMU turnIMU;

  private enum CalibrationState {
    IDLE,
    PULSE_FORWARD,
    SETTLE_FORWARD,
    PULSE_REVERSE,
    SETTLE_REVERSE,
    COMPLETE,
    FAILED
  }

  private CalibrationState state = CalibrationState.IDLE;
  private final ElapsedTime timer = new ElapsedTime();

  private final List<Double> voltages = new ArrayList<>();
  private final List<Double> imuAngles = new ArrayList<>();

  private double minVoltage = 3.3;
  private double maxVoltage = 0.0;

  // Calibration Results
  private double calculatedZeroVoltage = 0.0;
  private double calculatedDegreesPerVolt = 0.0;
  private boolean calculatedInverted = false;
  private double rSquared = 0.0;

  private static final double SWEEP_POWER = 0.25;
  private static final double MAX_SWEEP_ANGLE = 90.0; // +/- 90 degrees (180 deg total range)
  private static final double PULSE_DURATION_SEC = 0.10; // 100 ms pulse
  private static final double SETTLE_DURATION_SEC =
      0.25; // 250 ms wait to eliminate IMU/Analog sensor lag

  private String hardwareErrorMsg = null;

  @Override
  public void init() {
    config.reload();

    try {
      turnServo = hardwareMap.get(CRServo.class, "turn");
    } catch (Exception e) {
      hardwareErrorMsg = "Missing CRServo 'turn'";
    }

    try {
      turnAnalog = hardwareMap.get(AnalogInput.class, "turnanalog");
    } catch (Exception e) {
      hardwareErrorMsg = "Missing AnalogInput 'turnanalog'";
    }

    try {
      turnIMU = hardwareMap.get(IMU.class, "turnImu");
      turnIMU.resetYaw();
    } catch (Exception e) {
      if (hardwareErrorMsg == null) {
        hardwareErrorMsg = "Missing IMU 'turnImu'";
      }
    }

    telemetry.addLine("=== TURRET ANALOG CALIBRATION ===");
    if (hardwareErrorMsg != null) {
      telemetry.addLine("ERROR: " + hardwareErrorMsg);
      telemetry.addLine("Check Robot Configuration on Driver Station!");
    } else {
      telemetry.addLine("Sweeps turret +/-90 deg (180 deg total) with settling delays");
      telemetry.addLine("to eliminate IMU and Analog sensor latency.");
      telemetry.addLine("");
      telemetry.addLine("Press [A] to start calibration sweep");
      telemetry.addLine("Press [B] to cancel at any time");
    }
    telemetry.update();
  }

  @Override
  public void loop() {
    if (hardwareErrorMsg != null || turnAnalog == null || turnIMU == null || turnServo == null) {
      telemetry.addLine("=== CANNOT RUN CALIBRATION ===");
      telemetry.addLine(hardwareErrorMsg != null ? hardwareErrorMsg : "Hardware missing");
      telemetry.addLine("Check Robot Configuration XML on Driver Station!");
      telemetry.update();
      return;
    }

    double rawVoltage = turnAnalog.getVoltage();
    YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
    double currentImuAngle =
        orientation != null ? Math.toDegrees(orientation.getYaw(AngleUnit.RADIANS)) : 0.0;

    if (rawVoltage < minVoltage) minVoltage = rawVoltage;
    if (rawVoltage > maxVoltage) maxVoltage = rawVoltage;

    if (gamepad1.b) {
      stopTurret();
      state = CalibrationState.IDLE;
    }

    switch (state) {
      case IDLE:
        stopTurret();
        if (gamepad1.a) {
          voltages.clear();
          imuAngles.clear();
          minVoltage = rawVoltage;
          maxVoltage = rawVoltage;
          timer.reset();
          state = CalibrationState.PULSE_FORWARD;
        }
        break;

      case PULSE_FORWARD:
        setServoPower(SWEEP_POWER);
        if (timer.seconds() >= PULSE_DURATION_SEC) {
          stopTurret();
          timer.reset();
          state = CalibrationState.SETTLE_FORWARD;
        }
        break;

      case SETTLE_FORWARD:
        stopTurret();
        if (timer.seconds() >= SETTLE_DURATION_SEC) {
          // Record sample ONLY after motor is fully stopped and sensors have settled
          recordSample(rawVoltage, currentImuAngle);

          if (currentImuAngle >= MAX_SWEEP_ANGLE || voltages.size() > 40) {
            timer.reset();
            state = CalibrationState.PULSE_REVERSE;
          } else {
            timer.reset();
            state = CalibrationState.PULSE_FORWARD;
          }
        }
        break;

      case PULSE_REVERSE:
        setServoPower(-SWEEP_POWER);
        if (timer.seconds() >= PULSE_DURATION_SEC) {
          stopTurret();
          timer.reset();
          state = CalibrationState.SETTLE_REVERSE;
        }
        break;

      case SETTLE_REVERSE:
        stopTurret();
        if (timer.seconds() >= SETTLE_DURATION_SEC) {
          // Record sample ONLY after motor is fully stopped and sensors have settled
          recordSample(rawVoltage, currentImuAngle);

          if (currentImuAngle <= -MAX_SWEEP_ANGLE || voltages.size() > 80) {
            stopTurret();
            if (calculateLinearRegression()) {
              state = CalibrationState.COMPLETE;
            } else {
              state = CalibrationState.FAILED;
            }
          } else {
            timer.reset();
            state = CalibrationState.PULSE_REVERSE;
          }
        }
        break;

      case COMPLETE:
        stopTurret();
        displayCompleteTelemetry();
        break;

      case FAILED:
        stopTurret();
        telemetry.addLine("=== CALIBRATION FAILED ===");
        telemetry.addLine("Insufficient samples or negligible turret movement.");
        telemetry.addLine("Press [A] to retry");
        if (gamepad1.a) state = CalibrationState.IDLE;
        break;
    }

    if (state == CalibrationState.IDLE) {
      telemetry.addLine("=== READY FOR CALIBRATION ===");
      telemetry.addData("Current Voltage", String.format("%.3f V", rawVoltage));
      telemetry.addData("Current IMU Yaw", String.format("%.2f°", currentImuAngle));
      telemetry.addLine("Press [A] to begin +/-90° sweep with settling delays");
    } else if (state != CalibrationState.COMPLETE && state != CalibrationState.FAILED) {
      telemetry.addLine("=== CALIBRATING (STEP & SETTLE) ===");
      telemetry.addData("State", state);
      telemetry.addData("Samples Collected", voltages.size());
      telemetry.addData("Settled Voltage", String.format("%.3f V", rawVoltage));
      telemetry.addData("Settled IMU Yaw", String.format("%.2f°", currentImuAngle));
      telemetry.addLine("Press [B] to abort");
    }

    telemetry.update();
  }

  private void recordSample(double v, double angle) {
    voltages.add(v);
    imuAngles.add(angle);
  }

  private void setServoPower(double power) {
    if (power > 0) {
      turnServo.setDirection(CRServo.Direction.FORWARD);
      turnServo.setPower(Math.abs(power));
    } else if (power < 0) {
      turnServo.setDirection(CRServo.Direction.REVERSE);
      turnServo.setPower(Math.abs(power));
    } else {
      turnServo.setPower(0);
    }
  }

  private void stopTurret() {
    turnServo.setPower(0);
  }

  private boolean calculateLinearRegression() {
    int n = voltages.size();
    if (n < 6) return false;

    double sumV = 0, sumA = 0;
    for (int i = 0; i < n; i++) {
      sumV += voltages.get(i);
      sumA += imuAngles.get(i);
    }
    double avgV = sumV / n;
    double avgA = sumA / n;

    double num = 0, den = 0;
    for (int i = 0; i < n; i++) {
      double vDiff = voltages.get(i) - avgV;
      double aDiff = imuAngles.get(i) - avgA;
      num += vDiff * aDiff;
      den += vDiff * vDiff;
    }

    if (Math.abs(den) < 1e-6) return false;

    double slope = num / den; // degrees per volt (signed)
    if (Math.abs(slope) < 1.0) return false; // negligible movement

    calculatedInverted = slope < 0;
    calculatedDegreesPerVolt = Math.abs(slope);

    // V0 is voltage at angle = 0: A = slope * (V - V0) => V0 = avgV - (avgA / slope)
    calculatedZeroVoltage = avgV - (avgA / slope);

    // Calculate R^2 fit quality
    double ssTot = 0, ssRes = 0;
    for (int i = 0; i < n; i++) {
      double predictedA = slope * (voltages.get(i) - calculatedZeroVoltage);
      double actualA = imuAngles.get(i);
      ssTot += Math.pow(actualA - avgA, 2);
      ssRes += Math.pow(actualA - predictedA, 2);
    }
    rSquared = (ssTot > 0) ? Math.max(0, 1.0 - (ssRes / ssTot)) : 1.0;

    return true;
  }

  private void displayCompleteTelemetry() {
    telemetry.addLine("=== CALIBRATION SUCCESSFUL ===");
    telemetry.addData("Fit Quality (R^2)", String.format("%.4f", rSquared));
    telemetry.addLine("");
    telemetry.addLine("--- RECOMMENDED CONFIG.YAML VALUES ---");
    telemetry.addLine("turret:");
    telemetry.addLine("  analog_encoder:");
    telemetry.addLine("    enabled: true");
    telemetry.addLine(String.format("    zero_voltage: %.4f", calculatedZeroVoltage));
    telemetry.addLine(String.format("    degrees_per_volt: %.2f", calculatedDegreesPerVolt));
    telemetry.addLine(String.format("    min_voltage: %.3f", minVoltage));
    telemetry.addLine(String.format("    max_voltage: %.3f", maxVoltage));
    telemetry.addLine(String.format("    inverted: %b", calculatedInverted));
  }
}
