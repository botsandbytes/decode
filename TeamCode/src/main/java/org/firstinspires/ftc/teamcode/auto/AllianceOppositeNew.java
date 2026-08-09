package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pcurve;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@Configurable
public abstract class AllianceOppositeNew extends AllianceAutoBase<config.OppositeAuto> {

  private PathChain scorePreload;
  private PathChain grabPickup4, scorePickup4;
  private PathChain gatePark;

  protected AllianceOppositeNew(Alliance alliance) {
    super(alliance, config.OppositeAuto.class, "auto_poses.opposite");
  }

  @Override
  protected CommandBuilder buildAuto() {
    double constantPower = Shooter.constantPower();

    return sequential(
        // Preload
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, scorePreload),
        shoot(),

        // Grab pickup 4 → score (Cycle 1)
        intakeAndFollow(grabPickup4),
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, scorePickup4),
        shoot(),

        // Grab pickup 4 → score (Cycle 2)
        intakeAndFollow(grabPickup4),
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, scorePickup4),
        shoot(),

        // Grab pickup 4 → score (Cycle 3)
        intakeAndFollow(grabPickup4),
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, scorePickup4),
        shoot(),

        // Grab pickup 4 → score (Cycle 4)
        intakeAndFollow(grabPickup4),
        instant(() -> shooter.setTargetPower(constantPower)),
        follow(follower, scorePickup4),
        shoot(),

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
              shooter.setTargetPower(Shooter.constantPower());
              intake.run(1, config.transferPower);
            }),
        follow(follower, path));
  }

  @Override
  protected void buildPaths() {
    scorePreload = pline(config.start, config.score);
    gatePark = pline(config.score, config.park, 0.1);
    grabPickup4 = pcurve(config.score, config.pickup4Cp, config.pickup4End, 0.1);
    scorePickup4 = pline(config.pickup4End, config.score);
  }
}
