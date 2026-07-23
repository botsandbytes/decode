package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.config;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "!")
public class TeleOp extends TeleOpBase {

  private Command parkCommand;
  private Command scoreCommand;
  private Command drinkCommand;
  private Command intakeCommand;
  private Command aimAndShootCommand;
  private Command aimCommand;
  private Command shootManualCommand;
  private Command manualRevCommand;

  @Override
  protected void buildCommands() {
    parkCommand =
        Command.build().setStart(() -> follower.holdPoint(profile.parkPose())).requiring(follower);

    scoreCommand =
        Command.build()
            .setStart(
                () -> {
                  shooter.setHoodLongShotPosition();
                  follower.holdPoint(profile.scorePose());
                })
            .requiring(follower);

    drinkCommand =
        Command.build().setStart(() -> follower.holdPoint(profile.drinkPose())).requiring(follower);

    intakeCommand =
        Command.build()
            .setStart(() -> intake.run(1.0, 0.1))
            .setEnd(interrupted -> intake.stop())
            .requiring(intake);

    aimAndShootCommand =
        Command.build()
            .setStart(
                () -> {
                  follower.holdPoint(follower.getPose());
                  shotController.startShot(currentParams.launchPower(), true);
                })
            .setExecute(
                () -> {
                  if (currentParams.launchPower() > 0.7) {
                    shooter.setHoodLongShotPosition();
                  } else {
                    shooter.setHoodPosition(0);
                  }
                })
            .setDone(
                () ->
                    shotController.getElapsedTimeMs() > currentParams.waitTime()
                        || !sentinel.isLaunchAllowed(follower.getPose()))
            .setEnd(interrupted -> shotController.stopShot())
            .requiring(follower, shooter, turret, intake);

    aimCommand =
        Command.build()
            .setStart(() -> turret.setAimMode(Turret.AimMode.AIM_AT_GOAL))
            .setExecute(
                () -> {
                  if (currentParams.launchPower() > 0.7) {
                    shooter.setHoodLongShotPosition();
                  } else {
                    shooter.setHoodPosition(0);
                  }
                  shooter.setTargetPower(currentParams.launchPower());
                })
            .setEnd(
                interrupted -> {
                  turret.setHoldAngle(Math.toDegrees(follower.getHeading()));
                  turret.setAimMode(Turret.AimMode.HOLD);
                  shooter.setTargetPower(0.0);
                })
            .requiring(shooter, turret);

    shootManualCommand =
        Command.build()
            .setStart(() -> shotController.startShot(currentParams.launchPower(), false))
            .setDone(() -> shotController.getElapsedTimeMs() > currentParams.waitTime())
            .setEnd(interrupted -> shotController.stopShot())
            .requiring(shooter, intake);

    manualRevCommand =
        Command.build()
            .setStart(
                () -> {
                  intake.stop();
                  shooter.setTargetPower(0.6);
                })
            .setEnd(interrupted -> shooter.setTargetPower(0.0))
            .requiring(shooter);
  }

  @Override
  protected boolean hasActiveDriveOverride() {
    return Scheduler.isScheduled(parkCommand)
        || Scheduler.isScheduled(scoreCommand)
        || Scheduler.isScheduled(drinkCommand)
        || Scheduler.isScheduled(aimAndShootCommand);
  }

  @Override
  protected void onLoop() {
    boolean launchAllowed = sentinel.isLaunchAllowed(follower.getPose());
    boolean autoShoot = config.shooter.auto_shoot_mode;

    if (operator.a().onTrue()) {
      intakeCommand.schedule();
    }

    if (operator.b().onTrue() || operator.y().onTrue()) {
      intakeCommand.cancel();
      manualRevCommand.schedule();
    }

    if (operatorRightTrigger.onTrue() && launchAllowed) {
      if (autoShoot) {
        aimAndShootCommand.schedule();
      } else {
        shootManualCommand.schedule();
      }
    }
    if (operatorRightTrigger.onFalse()) {
      if (autoShoot) {
        aimAndShootCommand.cancel();
      } else {
        shootManualCommand.cancel();
      }
    }

    if (!autoShoot && operator.x().onTrue() && launchAllowed) {
      aimCommand.schedule();
    }
    if (!autoShoot && operator.x().onFalse()) {
      aimCommand.cancel();
    }

    if (operator.dpadUp().onTrue()) {
      shootManualCommand.schedule();
    }

    if (operatorLeftTrigger.onTrue()) {
      aimAndShootCommand.cancel();
      aimCommand.cancel();
      shootManualCommand.cancel();
      manualRevCommand.cancel();
    }

    if (driver.dpadLeft().onTrue()) {
      parkCommand.schedule();
    }
    if (driverRightTrigger.onTrue()) {
      scoreCommand.schedule();
    }
    if (driverLeftTrigger.onTrue()) {
      drinkCommand.schedule();
    }

    if (driver.dpadRight().onTrue() || operator.dpadRight().onTrue()) {
      parkCommand.cancel();
      scoreCommand.cancel();
      drinkCommand.cancel();
    }
  }
}
