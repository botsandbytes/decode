package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;

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

  /** Latched state of the driver's face-the-goal heading lock; toggled by driver X. */
  private boolean goalHeadingLockEngaged = false;

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

    // Built by ShotController so autonomous fires the exact same shot; see
    // ShotController#aimAndShootCommand.
    aimAndShootCommand = shotController.aimAndShootCommand(follower, sentinel);

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
            .setStart(() -> shotController.startShot(Shooter.constantPower(), false))
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

    // Driver X toggles the heading lock between facing the goal continuously and its default of
    // holding whatever heading the turn stick was last released at. While engaged it owns the turn
    // axis (translation is untouched); pressing X again or steering releases it, and the telemetry
    // line in TeleOpBase reports which mode is live.
    //
    // The target is republished every loop rather than latched once, so the bearing tracks the
    // robot as the driver translates. Deliberately the raw goal bearing, not ShotController's
    // lead-compensated azimuth: the lead offset is proportional to robot velocity, so folding it
    // in would make the chassis chase its own motion — the lock turns the robot, the turn adds
    // velocity, the target moves again.
    if (driverX.onTrue()) {
      goalHeadingLockEngaged = !goalHeadingLockEngaged;
    }

    // Steering releases the lock, so a latched mode can never leave the driver unable to turn.
    // Checked after the toggle above so reaching for the stick in the same loop as the press wins.
    // Reuses the configured "is the driver steering at all" threshold that the automatic heading
    // lock already unlatches on, rather than a second hand-picked deadband — the stick has to feel
    // the same in both modes. Deliberately turn-only: strafing while facing the goal is the point.
    if (Math.abs(driver.right_stick_x) > Casablanca.headingLockIntentThreshold) {
      goalHeadingLockEngaged = false;
    }

    Pose pose = follower.getPose();
    casablanca.setGoalHeadingLock(
        Turret.alignPose(pose.getX(), pose.getY(), profile.goalX(), profile.goalY()).getHeading(),
        goalHeadingLockEngaged);

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
