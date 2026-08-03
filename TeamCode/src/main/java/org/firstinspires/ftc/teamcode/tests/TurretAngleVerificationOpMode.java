package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@TeleOp(name = "Turret Angle Verification", group = "Tuning")
public class TurretAngleVerificationOpMode extends OpMode {

  private Turret turret;
  private IMU turnIMU;
  private AnalogInput turnAnalog;

  private double targetAngle = 0.0;
  private double startImuAngle = 0.0;

  private boolean dpadUpPrev = false;
  private boolean dpadDownPrev = false;
  private boolean dpadLeftPrev = false;
  private boolean dpadRightPrev = false;

  @Override
  public void init() {
    config.reload();

    turret = new Turret(hardwareMap, telemetry);
    turret.setGoal(100.0, 100.0);
    try {
      turnAnalog = hardwareMap.get(AnalogInput.class, "turnanalog");
    } catch (Exception ignored) {
    }

    try {
      turnIMU = hardwareMap.get(IMU.class, "turnImu");
      RevHubOrientationOnRobot.LogoFacingDirection logoDir =
          RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
      RevHubOrientationOnRobot.UsbFacingDirection usbDir =
          RevHubOrientationOnRobot.UsbFacingDirection.UP;

      if (config.turret != null && config.turret.orientation != null) {
        try {
          logoDir =
              RevHubOrientationOnRobot.LogoFacingDirection.valueOf(config.turret.orientation.logo);
          usbDir =
              RevHubOrientationOnRobot.UsbFacingDirection.valueOf(config.turret.orientation.usb);
        } catch (Exception ignored) {
        }
      }

      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logoDir, usbDir));
      turnIMU.initialize(parameters);
      turnIMU.resetYaw();
    } catch (Exception e) {
      telemetry.addData("IMU Error", e.getMessage());
    }

    startImuAngle = getRawImuAngle();

    telemetry.addLine("=== TURRET ANGLE VERIFICATION ===");
    telemetry.addLine("Compares Analog Encoder angle against IMU (Ground Truth).");
    telemetry.addLine("Controls:");
    telemetry.addLine("  D-Pad UP / DOWN : +/- 5° target angle");
    telemetry.addLine("  D-Pad LEFT / RIGHT : +/- 15° target angle");
    telemetry.addLine("  Button [Y] : Set target to 0° (Center)");
    telemetry.addLine("  Button [X] : Set target to +30°");
    telemetry.addLine("  Button [B] : Set target to -30°");
    telemetry.addLine("  Button [A] : Zero IMU Ground Truth reference");
    telemetry.update();
  }

  private boolean activeHolding = false;

  @Override
  public void start() {
    activeHolding = false;
    turret.setAimMode(Turret.AimMode.IDLE);
    turret.periodic();
  }

  private double getRawImuAngle() {
    if (turnIMU == null) return 0.0;
    YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
    return orientation != null ? Math.toDegrees(orientation.getYaw(AngleUnit.RADIANS)) : 0.0;
  }

  private boolean aPrev = false;
  private boolean bPrev = false;
  private boolean xPrev = false;
  private boolean yPrev = false;

  @SuppressLint("DefaultLocale")
  @Override
  public void loop() {
    // Gamepad controls for target angle
    if (gamepad1.dpad_up && !dpadUpPrev) {
      targetAngle += 5.0;
      activeHolding = true;
    }
    if (gamepad1.dpad_down && !dpadDownPrev) {
      targetAngle -= 5.0;
      activeHolding = true;
    }
    if (gamepad1.dpad_left && !dpadLeftPrev) {
      targetAngle += 15.0;
      activeHolding = true;
    }
    if (gamepad1.dpad_right && !dpadRightPrev) {
      targetAngle -= 15.0;
      activeHolding = true;
    }

    dpadUpPrev = gamepad1.dpad_up;
    dpadDownPrev = gamepad1.dpad_down;
    dpadLeftPrev = gamepad1.dpad_left;
    dpadRightPrev = gamepad1.dpad_right;

    if (gamepad1.y && !yPrev) {
      targetAngle = 0.0;
      activeHolding = true;
    }
    if (gamepad1.x && !xPrev) {
      targetAngle = 30.0;
      activeHolding = true;
    }
    if (gamepad1.b && !bPrev) {
      targetAngle = -30.0;
      activeHolding = true;
    }

    // Reset the IMU ground-truth reference and target angle to 0 when pressing A.
    //
    // Deliberately does NOT re-zero the analog encoder. zero_voltage is a calibration constant
    // measured from the mechanical hard stops; overwriting it with wherever the turret happens to
    // be sitting silently shifts the entire angle mapping. That is exactly what wrecked a run
    // here: a good calibration (zero 1.9285 V) was clobbered to 0.752 V by this button, shifting
    // every angle ~50 deg, so a "5 deg" target was physically past the mechanical stop. The turret
    // drove into it, skipped gears, and the potentiometer wrapped past its rail.
    if (gamepad1.a && !aPrev) {
      targetAngle = 0.0;
      if (turnIMU != null) turnIMU.resetYaw();
      startImuAngle = getRawImuAngle();
      activeHolding = true;
    }

    yPrev = gamepad1.y;
    xPrev = gamepad1.x;
    bPrev = gamepad1.b;
    aPrev = gamepad1.a;

    // Left Trigger releases active holding (turns off servo power / IDLE)
    if (gamepad1.left_trigger > 0.2) {
      activeHolding = false;
    }

    // Target angle safety clamp
    targetAngle = Math.clamp(targetAngle, -45.0, 45.0);

    // Apply target angle or stay idle
    if (activeHolding) {
      turret.setHoldAngle(targetAngle);
      turret.setAimMode(Turret.AimMode.HOLD);
    } else {
      turret.setAimMode(Turret.AimMode.IDLE);
    }
    turret.periodic();

    // Measure values
    double rawVoltage = turnAnalog != null ? turnAnalog.getVoltage() : 0.0;
    double analogCalculatedAngle = turret.getCurrentTurnAngle();
    double imuGroundTruthAngle = AngleUnit.normalizeDegrees(getRawImuAngle() - startImuAngle);
    double error = Math.abs(analogCalculatedAngle - imuGroundTruthAngle);

    telemetry.addLine("=== ANGLE COMPARISON (ANALOG vs IMU) ===");
    telemetry.addData("Servo Status", activeHolding ? "HOLDING AT TARGET" : "IDLE (POWER OFF)");
    telemetry.addData("Commanded Target", String.format("%.1f°", targetAngle));
    telemetry.addData("Analog Encoder Angle", String.format("%.2f°", analogCalculatedAngle));
    telemetry.addData("IMU Ground Truth", String.format("%.2f°", imuGroundTruthAngle));
    telemetry.addData("Encoder vs IMU Error", String.format("%.2f°", error));
    telemetry.addLine("--- SENSOR DATA ---");
    telemetry.addData("Raw Analog Voltage", String.format("%.4f V", rawVoltage));
    telemetry.addData("Zero Voltage (Turret)", String.format("%.4f V", turret.getZeroVoltage()));
    telemetry.addData(
        "Deg/Volt (Config)", String.format("%.2f", config.turret.analog_encoder.degrees_per_volt));
    telemetry.addLine("");
    telemetry.addLine("Press [Left Trigger] to kill servo power / IDLE");
    telemetry.update();
  }
}
