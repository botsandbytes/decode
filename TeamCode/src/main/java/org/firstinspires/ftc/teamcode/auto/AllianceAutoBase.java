package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.bylazar.field.FieldManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.Field;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Generic, season-agnostic autonomous base class encapsulating alliance state, hardware lifecycle,
 * dashboard field updates, telemetry, generic config binding, and shared commands.
 */
public abstract class AllianceAutoBase<T> extends OpMode {

  protected final Alliance alliance;
  protected final double goalX;
  protected final double goalY;

  protected final Class<T> configClass;
  protected final String posePrefix;
  protected T config;

  protected Robot robot;
  protected Intake intake;
  protected Shooter shooter;
  protected Turret turret;
  protected Follower follower;
  protected FieldManager field;

  protected AllianceAutoBase(Alliance alliance, Class<T> configClass, String posePrefix) {
    this.alliance = alliance;
    this.goalX = Field.getGoalX(alliance);
    this.goalY = Field.getGoalY(alliance);
    this.configClass = configClass;
    this.posePrefix = posePrefix;
  }

  protected abstract void buildPaths();

  protected abstract CommandBuilder buildAuto();

  private Pose extractStartPose(T config) {
    try {
      return (Pose) config.getClass().getField("start").get(config);
    } catch (Exception e) {
      return new Pose(0, 0, 0);
    }
  }

  @Override
  public void init() {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    String allianceStr = alliance == Alliance.RED ? "red" : "blue";
    this.config = ConfigLoader.loadMerged(configClass, posePrefix + "." + allianceStr, "auto");

    field = OpModeUtil.initPanelsField();

    blackboard.clear();
    blackboard.put("ALLIANCE", alliance);

    MatchProfile profile = new MatchProfile(alliance);

    robot = new Robot(hardwareMap, telemetry, profile);
    follower = robot.follower;
    intake = robot.intake;
    shooter = robot.shooter;
    turret = robot.turret;

    Pose startPose = extractStartPose(config);
    OpModeUtil.setupTurretAndShooter(turret, shooter, startPose.getHeading());

    follower.setStartingPose(startPose);
    buildPaths();
    Scheduler.reset();
  }

  @Override
  public void start() {
    schedule(buildAuto());
  }

  @Override
  public void loop() {
    turret.setHoldAngle(Math.toDegrees(follower.getHeading()));
    robot.update();

    OpModeUtil.savePose(alliance, follower.getPose());
    OpModeUtil.drawRobot(field, follower, turret, goalX, goalY);

    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.update();
  }
}
