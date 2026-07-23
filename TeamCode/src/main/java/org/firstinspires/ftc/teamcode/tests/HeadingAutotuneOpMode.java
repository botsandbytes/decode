package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utilities.AutotuneOpMode;
import org.firstinspires.ftc.teamcode.utilities.PIDAutotuner;

@TeleOp(name = "Chassis Heading Autotune", group = "Tuning")
public class HeadingAutotuneOpMode extends AutotuneOpMode {

  public HeadingAutotuneOpMode() {
    super("chassis heading", 0.25);
  }

  @Override
  protected PIDAutotuner createAutotuner() {
    return new PIDAutotuner();
  }

  @Override
  protected double readCurrentValue() {
    return follower.getHeading();
  }

  @Override
  protected void applyPower(double power) {
    follower.setTeleOpDrive(0.0, 0.0, power, true);
  }

  @Override
  protected void updateCustomCompleteTelemetry() {
    addGains("PESSEN INTEGRATION (STRONG HOLD)", autotuner.getPessen());
    telemetryM.addLine("");
    telemetryM.addLine("INSTRUCTIONS FOR ENGINEER:");
    telemetryM.addLine("Update 'casablanca.heading_lock.kp' in config.yaml");
    telemetryM.addLine("with the recommended Tyreus-Luyben Kp gain above.");
    telemetryM.addLine("");
  }
}
