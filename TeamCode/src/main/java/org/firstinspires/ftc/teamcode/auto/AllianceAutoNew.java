package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pcurve;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@Configurable
public abstract class AllianceAutoNew extends AllianceAutoBase<config.NormalAuto> {

  private PathChain scorePreload;
  private PathChain grabPickup1, scorePickup1;
  private PathChain drinkPickupStart, drinkPickupScore;
  private PathChain grabPickup2, scorePickup2;
  private PathChain grabPickup3, scorePickup3;

  protected AllianceAutoNew(Alliance alliance) {
    super(alliance, config.NormalAuto.class, "auto_poses.normal");
  }

  @Override
  protected CommandBuilder buildAuto() {
    double constantPower =
        org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.constant_rpm
            / org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.max_rpm;
    var shots = robot.shotController;

    return sequential(
        // Preload
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, scorePreload),
        shots.shootCommand(constantPower, config.shootWaitMs),

        // Line 2 pick up → score
        intakeAndFollow(grabPickup2),
        follow(follower, scorePickup2),
        shots.shootCommand(constantPower, config.shootWaitMs),

        // Drink gate round 1
        intakeAndFollow(drinkPickupStart),
        waitMs(config.drinkWaitMs),
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, drinkPickupScore),
        shots.shootCommand(constantPower, config.shootWaitMs),

        // Drink gate round 2
        intakeAndFollow(drinkPickupStart),
        waitMs(config.drinkWaitMs),
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, drinkPickupScore),
        shots.shootCommand(constantPower, config.shootWaitMs),

        // Line 1 pick up → score
        intakeAndFollow(grabPickup1),
        follow(follower, scorePickup1),
        shots.shootCommand(constantPower, config.shootWaitMs),

        // Line 3 pick up → score
        intakeAndFollow(grabPickup3),
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, scorePickup3),
        shots.shootCommand(constantPower, config.shootWaitMs),

        // Done
        instant(
            () -> {
              intake.stop();
              shooter.setTargetPower(0);
            }));
  }

  protected CommandBuilder intakeAndFollow(PathChain path) {
    double constantPower =
        org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.constant_rpm
            / org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.max_rpm;
    return sequential(
        instant(
            () -> {
              shooter.setTargetPower(constantPower);
              intake.run(1, config.transferPower);
            }),
        follow(follower, path));
  }

  @Override
  protected void buildPaths() {
    scorePreload = pline(config.start, config.score);
    drinkPickupStart = pcurve(config.score, config.drinkCp, config.drinkEnd);
    drinkPickupScore = pcurve(config.drinkEnd, config.drinkCp, config.score);
    grabPickup1 = pcurve(config.score, config.pickup1Cp, config.pickup1End, 0.1);
    scorePickup1 = pcurve(config.pickup1End, config.pickup1Cp, config.score);
    grabPickup2 = pcurve(config.score, config.pickup2Cp, config.pickup2End, 0.5);
    scorePickup2 = pcurve(config.pickup2End, config.pickup2Cp, config.score);
    grabPickup3 = pcurve(config.score, config.pickup3Cp, config.pickup3End, 0.5);
    scorePickup3 = pcurve(config.pickup3End, config.pickup3Cp, config.finalScore);
  }
}
