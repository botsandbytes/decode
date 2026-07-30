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
  private final String systemName;
  private final double outputMagnitude;
  private double currentMagnitude;
  private boolean autotuneStarted = false;
  private double targetValue = 0.0;
  private boolean lastAState = false;
  private boolean lastBState = false;
  private boolean lastDpadUpState = false;
  private boolean lastDpadDownState = false;

  protected AutotuneOpMode(String systemName, double outputMagnitude) {
    this.systemName = systemName;
    this.outputMagnitude = outputMagnitude;
    this.currentMagnitude = outputMagnitude;
  }

  @Override
  public void init() {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(0, 0, 0));

    OpModeUtil.initPanelsField();
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    autotuner = createAutotuner();
    currentMagnitude = outputMagnitude;
    autotuneStarted = false;

    telemetryM.addLine("=== " + systemName.toUpperCase() + " AUTOTUNE ===");
    telemetryM.addLine("Press [A] to start");
    telemetryM.addLine("Press [DPAD UP/DOWN] to adjust power (Current: " + currentMagnitude + ")");
    telemetryM.addLine("Press [B] to cancel");
    telemetryM.addLine(systemName + " will oscillate");
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

    boolean aJustPressed = gamepad1.a && !lastAState;
    boolean bJustPressed = gamepad1.b && !lastBState;
    boolean dpadUpJustPressed = gamepad1.dpad_up && !lastDpadUpState;
    boolean dpadDownJustPressed = gamepad1.dpad_down && !lastDpadDownState;

    lastAState = gamepad1.a;
    lastBState = gamepad1.b;
    lastDpadUpState = gamepad1.dpad_up;
    lastDpadDownState = gamepad1.dpad_down;

    if (dpadUpJustPressed && !autotuner.isRunning()) {
      currentMagnitude = Math.clamp(currentMagnitude + 0.05, 0.05, 1.0);
    }
    if (dpadDownJustPressed && !autotuner.isRunning()) {
      currentMagnitude = Math.clamp(currentMagnitude - 0.05, 0.05, 1.0);
    }

    if (bJustPressed) {
      autotuner.cancel();
      applyPower(0.0);
      autotuneStarted = false;
    }

    if (aJustPressed && !autotuner.isRunning()) {
      targetValue = currentVal;
      autotuner.startAutotune(currentVal, targetValue, currentMagnitude);
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
    telemetryM.addData("Output Power Magnitude", currentMagnitude);
    telemetryM.addLine("");
    telemetryM.addLine("Press [A] to start autotune");
    telemetryM.addLine("Press [DPAD UP/DOWN] to adjust power");
  }

  private void updateRunningTelemetry(double currentVal) {
    telemetryM.addLine("=== AUTOTUNING " + systemName.toUpperCase() + "... ===");
    telemetryM.addData("Target Value", targetValue);
    telemetryM.addData("Current Value", currentVal);
    telemetryM.addData("Output Power Magnitude", currentMagnitude);
    telemetryM.addData("Crossings", autotuner.getCrossingCount());
    telemetryM.addLine("");
    telemetryM.addLine("Press [B] to cancel");
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
    telemetryM.addLine("Ensure system can move freely and try increasing power with [DPAD UP]");
    telemetryM.addLine("");
    telemetryM.addLine("Press [A] to retry");
    autotuneStarted = false;
  }
}
