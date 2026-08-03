package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

public class ShotController {
  private final Shooter shooter;
  private final Turret turret;
  private final Intake intake;
  private final Supplier<Pose> poseSupplier;
  private final Telemetry telemetry;

  private boolean active = false;
  private double targetPower = 0.0;
  private boolean checkAlignment = false;
  private final ElapsedTime timer = new ElapsedTime();

  public ShotController(
      Shooter shooter,
      Turret turret,
      Intake intake,
      Supplier<Pose> poseSupplier,
      Telemetry telemetry) {
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    this.poseSupplier = poseSupplier;
    this.telemetry = telemetry;
  }

  public void startShot(double power, boolean checkAlignment) {
    this.active = true;
    this.targetPower = power;
    this.checkAlignment = checkAlignment;
    this.timer.reset();
    shooter.setTargetPower(power);
    if (checkAlignment) {
      turret.setAimMode(Turret.AimMode.AIM_AT_GOAL);
    }
  }

  public void stopShot() {
    this.active = false;
    this.targetPower = 0.0;
    shooter.setTargetPower(0.0);
    intake.stop();
    if (checkAlignment) {
      Pose pose = poseSupplier.get();
      if (pose != null) {
        turret.setHoldAngle(Math.toDegrees(pose.getHeading()));
      }
      turret.setAimMode(Turret.AimMode.HOLD);
    }
  }

  public CommandBuilder shootCommand(double power) {
    return shootCommand(power, config.auto.shoot_wait_ms);
  }

  public CommandBuilder shootCommand(double power, int shootWaitMs) {
    return sequential(
        instant(
            () -> {
              intake.stop();
              startShot(power, false);
            }),
        waitUntil(() -> getElapsedTimeMs() > shootWaitMs),
        instant(this::stopShot));
  }

  public boolean isActive() {
    return active;
  }

  public double getElapsedTimeMs() {
    return timer.milliseconds();
  }

  public void periodic() {
    if (!active) {
      return;
    }

    var shooterConfig = config.shooter;
    double targetFlywheelVelocity = shooterConfig.max_rpm * targetPower;
    double currentFlywheelVelocity = shooter.getShooterVelocity();

    double minTransferThreshold = shooterConfig.min_transfer_threshold;
    boolean isFlywheelAtMinSpeed =
        Math.abs(currentFlywheelVelocity)
            >= Math.abs(targetFlywheelVelocity) * minTransferThreshold;
    boolean isFlywheelReady = isFlywheelAtMinSpeed;
    boolean isTurretAligned = true;

    Pose pose = poseSupplier.get();
    if (checkAlignment && pose != null) {
      boolean isFlywheelBelowMaxThreshold =
          Math.abs(currentFlywheelVelocity)
              < Math.abs(targetFlywheelVelocity) * shooterConfig.max_velocity_threshold;
      isFlywheelReady = isFlywheelAtMinSpeed && isFlywheelBelowMaxThreshold;
      isTurretAligned = turret.isAimed(pose);

      if (telemetry != null) {
        telemetry.addData("ShotController State", "Aiming & Firing");
        telemetry.addData("current v", Math.abs(currentFlywheelVelocity));
        telemetry.addData(
            "target v threshold", Math.abs(targetFlywheelVelocity) * minTransferThreshold);
        telemetry.addData("Turret Error (deg)", turret.getAimError(pose));
        telemetry.addData("Turret Aligned", isTurretAligned);
      }
    } else {
      if (telemetry != null) {
        telemetry.addData("ShotController State", "Firing Raw");
        telemetry.addData("current v", Math.abs(currentFlywheelVelocity));
        telemetry.addData(
            "target v threshold", Math.abs(targetFlywheelVelocity) * minTransferThreshold);
      }
    }

    if (isFlywheelReady && isTurretAligned) {
      intake.run(shooterConfig.feed_intake_power, shooterConfig.feed_transfer_power);
    } else {
      intake.stop();
    }
  }
}
