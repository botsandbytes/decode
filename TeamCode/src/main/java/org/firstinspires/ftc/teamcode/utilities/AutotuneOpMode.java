package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.records.PIDGains;

public abstract class AutotuneOpMode extends OpMode {
  protected Follower follower;
  protected PIDAutotuner autotuner;
  protected TelemetryManager telemetryM;
  private List<LynxModule> allHubs;
  private boolean autotuneStarted = false;
  private double targetValue = 0.0;
  private final String systemName;
  private final double outputMagnitude;

  protected AutotuneOpMode(String systemName, double outputMagnitude) {
    this.systemName = systemName;
    this.outputMagnitude = outputMagnitude;
  }

  @Override
  public void init() {
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(0, 0, 0));

    OpModeUtil.initPanelsField();
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    autotuner = createAutotuner();

    telemetryM.addLine("=== " + systemName.toUpperCase() + " AUTOTUNE ===");
    telemetryM.addLine("Press [A] to start");
    telemetryM.addLine(systemName + " will oscillate");
    telemetryM.addLine("");
    telemetryM.addLine("Make sure " + systemName + " can move freely!");
    telemetryM.update();
  }

  protected abstract PIDAutotuner createAutotuner();

  protected abstract double readCurrentValue();

  protected abstract void applyPower(double power);

  protected void addGains(String name, PIDGains gains) {
    telemetryM.addLine("--- " + name + " ---");
    telemetryM.addData("Kp", gains.kp());
    telemetryM.addData("Ki", gains.ki());
    telemetryM.addData("Kd", gains.kd());
  }

  protected abstract void updateCustomCompleteTelemetry();

  @Override
  public void start() {
    follower.startTeleopDrive();
  }

  @Override
  public void loop() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }

    follower.update();
    double currentVal = readCurrentValue();

    if (gamepad1.a && !autotuneStarted) {
      targetValue = currentVal;
      autotuner.startAutotune(currentVal, targetValue, outputMagnitude);
      autotuneStarted = true;
    }

    if (autotuner.isRunning()) {
      double power = autotuner.updateAutotune(currentVal);
      applyPower(power);
      updateRunningTelemetry(currentVal);
    } else if (autotuner.isComplete()) {
      applyPower(0.0);
      updateCompleteTelemetry();
    } else if (autotuner.isFailed()) {
      applyPower(0.0);
      updateFailedTelemetry();
    } else {
      updateIdleTelemetry(currentVal);
    }

    telemetryM.update();
  }

  private void updateIdleTelemetry(double currentVal) {
    telemetryM.addLine("=== READY ===");
    telemetryM.addData("Current " + systemName + " Value", currentVal);
    telemetryM.addLine("");
    telemetryM.addLine("Press [A] to start autotune");
  }

  private void updateRunningTelemetry(double currentVal) {
    telemetryM.addLine("=== AUTOTUNING " + systemName.toUpperCase() + "... ===");
    telemetryM.addData("Target Value", targetValue);
    telemetryM.addData("Current Value", currentVal);
    telemetryM.addData("Crossings", autotuner.getCrossingCount());
    telemetryM.addLine("");
    telemetryM.addLine("Wait for oscillations...");
  }

  private void updateCompleteTelemetry() {
    telemetryM.addLine("=== COMPLETE ===");
    telemetryM.addLine("");
    addGains("TYREUS-LUYBEN (RECOMMENDED)", autotuner.getTyreusLuyben());
    telemetryM.addLine("");
    addGains("ZIEGLER-NICHOLS (AGGRESSIVE)", autotuner.getZieglerNichols());
    telemetryM.addLine("");
    updateCustomCompleteTelemetry();
    telemetryM.addData("Ku", autotuner.getKu());
    telemetryM.addData("Pu", autotuner.getPu());
  }

  private void updateFailedTelemetry() {
    telemetryM.addLine("=== FAILED ===");
    telemetryM.addLine("");
    telemetryM.addLine("Not enough oscillations (timeout)");
    telemetryM.addLine("Ensure system can move freely");
    telemetryM.addLine("");
    telemetryM.addLine("Press [A] to retry");
    autotuneStarted = false;
  }
}
