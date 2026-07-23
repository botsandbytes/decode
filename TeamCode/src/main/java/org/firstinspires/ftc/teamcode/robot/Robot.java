package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;

public final class Robot {

  public final Follower follower;
  public final Intake intake;
  public final Shooter shooter;
  public final Turret turret;
  public final VisionUtil vision;
  public final Sentinel sentinel;
  public final Casablanca casablanca;
  public final ShotController shotController;
  public final List<LynxModule> allHubs;
  public final Telemetry telemetry;

  public Robot(HardwareMap hardwareMap, Telemetry telemetry, MatchProfile profile) {
    this.telemetry = telemetry;

    // Bulk-caching setup
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    // Initialize Follower first (drivetrain motors caching is inside
    // Constants.createCachedFollower)
    follower = Constants.createCachedFollower(hardwareMap);

    // Initialize subsystems
    intake = new Intake(hardwareMap);
    shooter = new Shooter(hardwareMap);
    turret = new Turret(hardwareMap, telemetry, follower::getPose);
    turret.setGoal(profile.goalX(), profile.goalY());

    // Initialize safety and shooting controller
    sentinel = new Sentinel(profile.alliance());
    casablanca = new Casablanca(sentinel);
    shotController = new ShotController(shooter, turret, intake, follower::getPose, telemetry);

    vision = new VisionUtil();
  }

  public void update() {
    // Clear bulk cache for the loop
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }

    // Update follower
    follower.update();

    // Run subsystem and controller periodic updates
    intake.periodic();
    shooter.periodic();
    turret.periodic();
    shotController.periodic();

    Scheduler.execute();
  }
}
