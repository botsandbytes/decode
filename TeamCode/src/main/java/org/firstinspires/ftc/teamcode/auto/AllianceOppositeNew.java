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
import org.firstinspires.ftc.teamcode.robot.config.config;

/**
 * Shared "score preload, cycle lines 2-4 plus the drink gate, then park" autonomous, run from the
 * starting wall opposite {@link AllianceAutoNew}. {@link BlueOppositeNew} and {@link
 * RedOppositeNew} supply the alliance-mirrored field poses.
 */
@Configurable
public abstract class AllianceOppositeNew extends AllianceAutoBase<config.OppositeAuto> {

  private PathChain scorePreload;
  private PathChain grabPickup4, scorePickup4;
  private PathChain drinkPickupStart, drinkPickupScore;
  private PathChain grabPickup2, scorePickup2;
  private PathChain grabPickup3, scorePickup3;
  private PathChain gatePark;

  protected AllianceOppositeNew(Alliance alliance) {
    super(alliance, config.OppositeAuto.class, "auto_poses.opposite");
  }

  @Override
  protected CommandBuilder buildAuto() {
    double basePower = config.launchPower;
    var shots = robot.shotController;

    return sequential(
        // Preload
        instant(() -> shooter.setTargetPower(basePower + 0.02)),
        follow(follower, scorePreload),
        shots.shootCommand(basePower, config.shootWaitMs),

        // Grab pickup 2 → score
        intakeAndFollow(grabPickup2),
        instant(() -> shooter.setTargetPower(basePower + 0.02)),
        follow(follower, scorePickup2),
        shots.shootCommand(basePower, config.shootWaitMs),

        // Drink gate
        intakeAndFollow(drinkPickupStart),
        waitMs(config.drinkWaitMs),
        instant(() -> shooter.setTargetPower(basePower + 0.02)),
        follow(follower, drinkPickupScore),
        shots.shootCommand(basePower, config.shootWaitMs),

        // Grab pickup 3 → score
        intakeAndFollow(grabPickup3),
        instant(() -> shooter.setTargetPower(basePower)),
        follow(follower, scorePickup3),
        shots.shootCommand(basePower, config.shootWaitMs),

        // Grab pickup 4 → score
        intakeAndFollow(grabPickup4),
        instant(() -> shooter.setTargetPower(basePower + 0.02)),
        follow(follower, scorePickup4),
        shots.shootCommand(basePower, config.shootWaitMs),

        // Park
        follow(follower, gatePark),
        instant(
            () -> {
              intake.stop();
              shooter.setTargetPower(0);
            }));
  }

  protected CommandBuilder intakeAndFollow(PathChain path) {
    return sequential(
        instant(
            () -> {
              shooter.setTargetPower(config.launchPower / 2);
              intake.run(1, config.transferPower);
            }),
        follow(follower, path));
  }

  @Override
  protected void buildPaths() {
    scorePreload = pline(config.start, config.score);
    drinkPickupStart = pcurve(config.score, config.drinkCp, config.drinkEnd);
    drinkPickupScore = pcurve(config.drinkEnd, config.drinkCp, config.score);
    gatePark = pline(config.score, config.park, 0.1);
    grabPickup4 = pcurve(config.score, config.pickup4Cp, config.pickup4End, 0.1);
    scorePickup4 = pline(config.pickup4End, config.score);
    grabPickup2 = pcurve(config.score, config.pickup2Cp, config.pickup2End, 0.5);
    scorePickup2 = pcurve(config.pickup2End, config.pickup2Cp, config.score);
    grabPickup3 = pcurve(config.score, config.pickup3Cp, config.pickup3End, 0.3);
    scorePickup3 = pline(config.pickup3End, config.score);
  }
}
