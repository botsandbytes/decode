package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Friction Calibration TeleOp", group = "Calibration")
@Configurable
public class FrictionCalibrationOpMode extends LinearOpMode {

  private Follower follower;
  private List<LynxModule> allHubs;

  private double ksX = 0.0;
  private double ksY = 0.0;
  private double ksRot = 0.0;

  @Override
  public void runOpMode() throws InterruptedException {
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    follower = Constants.createFollower(hardwareMap);
    follower.startTeleopDrive();

    telemetry.addLine("Friction Auto-Calibration initialized.");
    telemetry.addLine("Place robot on open carpet with plenty of space!");
    telemetry.addLine("---------------------------------------------");
    telemetry.addLine("Buttons:");
    telemetry.addLine("  A (Cross)  -> Tune Forward Friction (kS X)");
    telemetry.addLine("  B (Circle) -> Tune Strafe Friction (kS Y)");
    telemetry.addLine("  X (Square) -> Tune Rotation Friction (kS Rot)");
    telemetry.update();

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      telemetry.addLine("Select a test:");
      telemetry.addLine("  A (Cross)  -> Tune Forward Friction (kS X)");
      telemetry.addLine("  B (Circle) -> Tune Strafe Friction (kS Y)");
      telemetry.addLine("  X (Square) -> Tune Rotation Friction (kS Rot)");
      telemetry.addLine("---------------------------------------------");
      telemetry.addData("Last Results:", "X: %.4f | Y: %.4f | Rot: %.4f", ksX, ksY, ksRot);
      telemetry.update();

      if (gamepad1.a) {
        runForwardTest();
      } else if (gamepad1.b) {
        runStrafeTest();
      } else if (gamepad1.x) {
        runRotationTest();
      }

      idle();
    }
  }

  private void runForwardTest() throws InterruptedException {
    telemetry.addLine("Running Forward (kS X) Test...");
    telemetry.update();
    sleep(500);

    double power = 0.0;
    double threshold = 0.15; // inches per second velocity threshold

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      power += 0.0003;
      follower.setTeleOpDrive(power, 0.0, 0.0, true);

      double currentVelocity = follower.getVelocity().getMagnitude();

      telemetry.addLine("FORWARD TEST (Ramping Power)");
      telemetry.addData("Current Power", "%.4f", power);
      telemetry.addData("Current Velocity", "%.4f in/s", currentVelocity);
      telemetry.addData("Threshold", "%.2f in/s", threshold);
      telemetry.update();

      if (currentVelocity > threshold) {
        ksX = power;
        break;
      }
      sleep(10);
    }

    stopRobot();
  }

  private void runStrafeTest() throws InterruptedException {
    telemetry.addLine("Running Strafe (kS Y) Test...");
    telemetry.update();
    sleep(500);

    double power = 0.0;
    double threshold = 0.15; // inches per second velocity threshold

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      power += 0.0003;
      follower.setTeleOpDrive(0.0, power, 0.0, true);

      double currentVelocity = follower.getVelocity().getMagnitude();

      telemetry.addLine("STRAFE TEST (Ramping Power)");
      telemetry.addData("Current Power", "%.4f", power);
      telemetry.addData("Current Velocity", "%.4f in/s", currentVelocity);
      telemetry.addData("Threshold", "%.2f in/s", threshold);
      telemetry.update();

      if (currentVelocity > threshold) {
        ksY = power;
        break;
      }
      sleep(10);
    }

    stopRobot();
  }

  private void runRotationTest() throws InterruptedException {
    telemetry.addLine("Running Rotation (kS Rot) Test...");
    telemetry.update();
    sleep(500);

    double power = 0.0;
    double threshold = 0.01; // radians per second velocity threshold

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      power += 0.0003;
      follower.setTeleOpDrive(0.0, 0.0, power, true);

      double currentVelocity = Math.abs(follower.getAngularVelocity());

      telemetry.addLine("ROTATION TEST (Ramping Power)");
      telemetry.addData("Current Power", "%.4f", power);
      telemetry.addData("Current Velocity", "%.4f rad/s", currentVelocity);
      telemetry.addData("Threshold", "%.2f rad/s", threshold);
      telemetry.update();

      if (currentVelocity > threshold) {
        ksRot = power;
        break;
      }
      sleep(10);
    }

    stopRobot();
  }

  private void stopRobot() throws InterruptedException {
    follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
    follower.update();

    telemetry.addLine("Test Complete! Stopping robot...");
    telemetry.update();

    while (opModeIsActive()
        && (follower.getVelocity().getMagnitude() > 0.05
            || Math.abs(follower.getAngularVelocity()) > 0.005)) {
      clearBulkCache();
      follower.update();
      follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
      sleep(10);
    }

    sleep(500);
  }

  private void clearBulkCache() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }
  }
}
