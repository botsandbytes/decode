package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

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
            .setStart(() -> intake.run(config.teleop.intake_power, config.teleop.transfer_power))
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
                  if (currentParams.launchPower() > config.shooter.long_hood_power_threshold) {
                    shooter.setHoodLongShotPosition();
                  } else {
                    shooter.setHoodShortShotPosition();
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
                  if (currentParams.launchPower() > config.shooter.long_hood_power_threshold) {
                    shooter.setHoodLongShotPosition();
                  } else {
                    shooter.setHoodShortShotPosition();
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
                  shooter.setTargetPower(config.teleop.manual_rev_power);
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

    if (operatorA.onTrue()) {
      intakeCommand.schedule();
    }

    if (operatorB.onTrue() || operatorY.onTrue()) {
      intakeCommand.cancel();
      manualRevCommand.schedule();
    }

    if (operatorRightTrigger.onTrue() && launchAllowed) {
      aimAndShootCommand.schedule();
    }
    if (operatorRightTrigger.onFalse()) {
      aimAndShootCommand.cancel();
    }

    if (operatorX.onTrue() && launchAllowed) {
      aimCommand.schedule();
    }
    if (operatorX.onFalse()) {
      aimCommand.cancel();
    }

    if (operatorDpadUp.onTrue()) {
      shootManualCommand.schedule();
    }

    if (operatorLeftTrigger.onTrue()) {
      aimAndShootCommand.cancel();
      aimCommand.cancel();
      shootManualCommand.cancel();
      manualRevCommand.cancel();
    }

    if (driverDpadLeft.onTrue()) {
      parkCommand.schedule();
    }
    if (driverRightTrigger.onTrue()) {
      scoreCommand.schedule();
    }
    if (driverLeftTrigger.onTrue()) {
      drinkCommand.schedule();
    }

    boolean driverStickDeflected =
        Math.abs(driver.left_stick_y) > 0.2
            || Math.abs(driver.left_stick_x) > 0.2
            || Math.abs(driver.right_stick_x) > 0.2;

    if (driverDpadRight.onTrue() || operatorDpadRight.onTrue() || driverStickDeflected) {
      parkCommand.cancel();
      scoreCommand.cancel();
      drinkCommand.cancel();
    }
  }
}
