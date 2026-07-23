package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretAutotuner;
import org.firstinspires.ftc.teamcode.utilities.AutotuneOpMode;
import org.firstinspires.ftc.teamcode.utilities.PIDAutotuner;

@TeleOp(name = "Turret Autotune", group = "Tuning")
public class TurretAutotuneOpMode extends AutotuneOpMode {
  private Turret turret;

  public TurretAutotuneOpMode() {
    super("turret", 0.3);
  }

  @Override
  public void init() {
    super.init();
    turret = new Turret(hardwareMap, telemetry, follower);
  }

  @Override
  protected PIDAutotuner createAutotuner() {
    return new TurretAutotuner();
  }

  @Override
  protected double readCurrentValue() {
    return turret.getCurrentTurnAngle();
  }

  @Override
  protected void applyPower(double power) {
    turret.setAimMode(Turret.AimMode.MANUAL);
    turret.setManualPower(power);
    turret.periodic();
  }

  @SuppressLint("DefaultLocale")
  @Override
  protected void updateCustomCompleteTelemetry() {
    telemetryM.addLine("RECOMMENDED (Tyreus-Luyben):");
    telemetryM.addLine(
        String.format(
            "pidf(%.4f, %.6f, %.6f, 0);",
            autotuner.getTyreusLuyben().kp(),
            autotuner.getTyreusLuyben().ki(),
            autotuner.getTyreusLuyben().kd()));
    telemetryM.addLine("Keep: double f = 0.08;");
    telemetryM.addLine("");
    telemetryM.addLine("INSTRUCTIONS FOR ENGINEER:");
    telemetryM.addLine("Update 'turret.pidf.p', 'turret.pidf.i', and 'turret.pidf.d'");
    telemetryM.addLine("in config.yaml with the recommended coefficients above.");
    telemetryM.addLine("");
  }
}
