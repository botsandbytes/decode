package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@Configurable
@TeleOp(name = "Hood Tuner", group = "Tuning")
public class HoodTunerOpMode extends OpMode {

  public static double targetPosition = 0.5;

  private Servo hood;
  private boolean dpadUpPressed = false;
  private boolean dpadDownPressed = false;
  private boolean dpadRightPressed = false;
  private boolean dpadLeftPressed = false;

  @Override
  public void init() {
    config.reload();
    hood = hardwareMap.get(Servo.class, "hood");
    targetPosition = config.shooter.hood.low_position;
    hood.setPosition(targetPosition);

    telemetry.addData("Status", "Initialized Hood Tuner");
    telemetry.addData("Config Low Position", config.shooter.hood.low_position);
    telemetry.addData("Config High Position", config.shooter.hood.high_position);
    telemetry.update();
  }

  @SuppressLint("DefaultLocale")
  @Override
  public void loop() {
    if (gamepad1.dpad_up && !dpadUpPressed) {
      targetPosition += 0.01;
    }
    dpadUpPressed = gamepad1.dpad_up;

    if (gamepad1.dpad_down && !dpadDownPressed) {
      targetPosition -= 0.01;
    }
    dpadDownPressed = gamepad1.dpad_down;

    if (gamepad1.dpad_right && !dpadRightPressed) {
      targetPosition += 0.001;
    }
    dpadRightPressed = gamepad1.dpad_right;

    if (gamepad1.dpad_left && !dpadLeftPressed) {
      targetPosition -= 0.001;
    }
    dpadLeftPressed = gamepad1.dpad_left;

    if (gamepad1.a) {
      targetPosition = config.shooter.hood.low_position;
    }
    if (gamepad1.b) {
      targetPosition = config.shooter.hood.high_position;
    }

    targetPosition = Math.clamp(targetPosition, 0.0, 1.0);
    hood.setPosition(targetPosition);

    telemetry.addData("Target Servo Position", String.format("%.4f", targetPosition));
    telemetry.addData("Actual Servo Position", String.format("%.4f", hood.getPosition()));
    telemetry.addData("Configured Low Position", config.shooter.hood.low_position);
    telemetry.addData("Configured High Position", config.shooter.hood.high_position);
    telemetry.addLine();
    telemetry.addLine("Controls:");
    telemetry.addLine("  DPAD Up/Down   : +/- 0.01");
    telemetry.addLine("  DPAD Right/Left: +/- 0.001");
    telemetry.addLine("  Button A       : Move to Config Low Position");
    telemetry.addLine("  Button B       : Move to Config High Position");
    telemetry.update();
  }
}
