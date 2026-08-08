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

    double constantPower = config.shooter.constant_rpm / config.shooter.max_rpm;

    aimAndShootCommand =
        Command.build()
            .setStart(
                () -> {
                  follower.holdPoint(follower.getPose());
                  shotController.startShot(constantPower, true);
                })
            .setDone(() -> !sentinel.isLaunchAllowed(follower.getPose()))
            .setEnd(interrupted -> shotController.stopShot())
            .requiring(follower, shooter, turret, intake);

    // Turret only, no flywheel. Aim (operator Y) and rev (operator X) are deliberately separate
    // commands over disjoint subsystems so holding both composes into aim-and-spin-up instead of
    // one overriding the other.
    aimCommand =
        Command.build()
            .setStart(() -> turret.setAimMode(Turret.AimMode.AIM_AT_GOAL))
            .setEnd(
                interrupted -> {
                  turret.setHoldAngle(0.0);
                  turret.setAimMode(Turret.AimMode.HOLD);
                })
            .requiring(turret);

    shootManualCommand =
        Command.build()
            .setStart(() -> shotController.startShot(constantPower, false))
            .setEnd(interrupted -> shotController.stopShot())
            .requiring(shooter, intake);

    // Flywheel only, no turret and no intake — intake ownership stays with intakeCommand so this
    // does not write to a subsystem it never required.
    manualRevCommand =
        Command.build()
            .setStart(() -> shooter.setTargetPower(config.teleop.manual_rev_power))
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

    // A starts the intake, B stops it. B used to be a second copy of the rev button, which left no
    // dedicated way to stop the intake once rev moved onto its own button.
    if (operatorA.onTrue()) {
      intakeCommand.schedule();
    }
    if (operatorB.onTrue()) {
      intakeCommand.cancel();
    }

    if (operatorRightTrigger.onTrue() && launchAllowed) {
      aimAndShootCommand.schedule();
    }
    if (operatorRightTrigger.onFalse()) {
      aimAndShootCommand.cancel();
    }

    // Y aims (turret), X revs (flywheel). Both are hold-to-run; holding both gives the old
    // combined aim-and-spin-up behavior that X alone used to have.
    if (operatorY.onTrue() && launchAllowed) {
      aimCommand.schedule();
    }
    if (operatorY.onFalse()) {
      aimCommand.cancel();
    }

    if (operatorX.onTrue()) {
      manualRevCommand.schedule();
    }
    if (operatorX.onFalse()) {
      manualRevCommand.cancel();
    }

    if (operatorDpadUp.onTrue()) {
      shootManualCommand.schedule();
    }

    if (operatorLeftTrigger.onTrue()) {
      aimAndShootCommand.cancel();
      aimCommand.cancel();
      shootManualCommand.cancel();
      manualRevCommand.cancel();
      intakeCommand.cancel();
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
