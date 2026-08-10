package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;

import com.bylazar.field.FieldManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.EndgameSpot;
import org.firstinspires.ftc.teamcode.records.Field;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config.Auto;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

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
  protected Sentinel sentinel;
  protected FieldManager field;

  private final ElapsedTime autoTimer = new ElapsedTime();
  private Command autoCommand;
  private boolean evacuating;
  private boolean frozen;
  private boolean shootingOut;
  private String endgameStatus = "scoring";

  protected AllianceAutoBase(Alliance alliance, Class<T> configClass, String posePrefix) {
    this.alliance = alliance;
    this.goalX = Field.getGoalX(alliance);
    this.goalY = Field.getGoalY(alliance);
    this.configClass = configClass;
    this.posePrefix = posePrefix;
  }

  protected abstract void buildPaths();

  protected abstract CommandBuilder buildAuto();

  /**
   * One scoring cycle's shot: the same aimed shot TeleOp's operator fires, capped at the window
   * measured for whatever distance the robot is standing at when it starts — see {@link
   * ShotController#timedAimAndShootCommand(Follower, Sentinel)}.
   */
  protected CommandBuilder shoot() {
    return robot.shotController.timedAimAndShootCommand(follower, sentinel);
  }

  private Pose extractStartPose(T config) {
    try {
      return (Pose) config.getClass().getField("start").get(config);
    } catch (Exception e) {
      return new Pose(0, 0, 0);
    }
  }

  private Pose extractScorePose(T config) {
    try {
      return (Pose) config.getClass().getField("score").get(config);
    } catch (Exception e) {
      return null;
    }
  }

  private void primeShooterForScorePose(T config) {
    Pose scorePose = extractScorePose(config);
    if (scorePose == null) {
      return;
    }

    org.firstinspires.ftc.teamcode.records.ShotInputs inputs =
        new org.firstinspires.ftc.teamcode.records.ShotInputs(
            scorePose, new Pose(0, 0, 0), goalX, goalY);
    org.firstinspires.ftc.teamcode.records.ShotSolution solution =
        org.firstinspires.ftc.teamcode.ballistics.ShotSolver.solve(
            inputs,
            org.firstinspires.ftc.teamcode.ballistics.ShotTable.fromConfig(),
            org.firstinspires.ftc.teamcode.records.BallisticsParameters.fromConfig(),
            0.4);
    robot.shotController.setInitialSolution(solution);
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
    sentinel = robot.sentinel;

    Pose startPose = extractStartPose(config);
    OpModeUtil.setupTurretAndShooter(turret, shooter);
    primeShooterForScorePose(config);

    follower.setStartingPose(startPose);
    buildPaths();
    Scheduler.reset();
  }

  @Override
  public void start() {
    evacuating = false;
    frozen = false;
    shootingOut = false;
    endgameStatus = "scoring";
    autoCommand = buildAuto();
    autoTimer.reset();
    schedule(autoCommand);
  }

  @Override
  public void loop() {
    updateEndgame();
    robot.update();

    OpModeUtil.savePose(alliance, follower.getPose());
    OpModeUtil.drawRobot(field, follower, turret, goalX, goalY);

    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.addData(
        "Endgame",
        "%s  (%.1f s left)",
        endgameStatus,
        Math.max(0, endgame().period_ms - autoTimer.milliseconds()) / 1000.0);
    addShooterDiagnostics();
    telemetry.update();
  }

  private static Auto.Endgame endgame() {
    return org.firstinspires.ftc.teamcode.robot.config.generated.config.auto.endgame;
  }

  private void updateEndgame() {
    double remainingMs = endgame().period_ms - autoTimer.milliseconds();
    if (!evacuating && remainingMs <= endgame().evacuate_ms) {
      beginEvacuation();
    }
    if (!evacuating) {
      return;
    }

    Sentinel.ZoneStanding standing =
        sentinel.zoneStanding(follower.getPose(), endgame().exit_clearance);

    if (!frozen
        && (standing != Sentinel.ZoneStanding.ON_BOUNDARY || remainingMs <= endgame().freeze_ms)) {
      freeze(standing);
    }
    if (frozen) {
      endgameStatus = (shootingOut ? "frozen, shooting - " : "frozen - ") + standing;
      if (!shootingOut) {
        follower.setTeleOpDrive(0, 0, 0);
      }
    }
  }

  private void beginEvacuation() {
    evacuating = true;

    Scheduler.cancel(autoCommand);
    Scheduler.reset();
    robot.shotController.stopShot();
    intake.stop();
    turret.setAimMode(Turret.AimMode.IDLE);
    follower.breakFollowing();

    Pose here = follower.getPose();
    EndgameSpot spot = sentinel.nearestEndgameSpot(here, endgame().exit_clearance);

    if (spot == null) {
      endgameStatus = "no committed spot on our half";
      shooter.setTargetPower(0);
      freeze(sentinel.zoneStanding(here, endgame().exit_clearance));
      return;
    }

    shooter.setTargetPower(spot.insideLaunchZone() ? Shooter.constantPower() : 0);

    endgameStatus =
        String.format(
            Locale.ROOT,
            "moving %.1f in to (%.0f, %.0f), %s",
            here.distanceFrom(spot.pose()),
            spot.pose().getX(),
            spot.pose().getY(),
            spot.insideLaunchZone() ? "inside zone" : "outside zones");
    schedule(follow(follower, pline(here, spot.pose())));
  }

  private void freeze(Sentinel.ZoneStanding standing) {
    frozen = true;
    Scheduler.reset();
    intake.stop();
    robot.shotController.stopShot();
    follower.breakFollowing();

    shootingOut = standing == Sentinel.ZoneStanding.INSIDE;
    if (shootingOut) {
      schedule(robot.shotController.aimAndShootCommand(follower, sentinel));
      return;
    }

    shooter.setTargetPower(0);
    turret.setAimMode(Turret.AimMode.IDLE);
    follower.startTeleopDrive();
    follower.setTeleOpDrive(0, 0, 0);
  }

  private void addShooterDiagnostics() {
    double target = shooter.getLastTargetVelocity();
    double actual = Math.abs(shooter.getShooterVelocity());
    telemetry.addData(
        "Flywheel",
        String.format(
            Locale.ROOT,
            "%.0f / %.0f ticks/s (%.2fx)",
            actual,
            target,
            target > 0 ? actual / target : 0.0));
    telemetry.addData(
        "Flywheel Cmd",
        String.format(
            Locale.ROOT,
            "%.3f  (pid %.3f + i %.3f + ff %.3f) x%.2f",
            shooter.getLastCommand(),
            shooter.getLastPidTerm(),
            shooter.getLastIntegralTerm(),
            shooter.getLastFeedforwardTerm(),
            shooter.getLastVoltageScale()));
    telemetry.addData(
        "Bus Voltage", String.format(Locale.ROOT, "%.2f V", shooter.getLastBusVoltage()));
    telemetry.addData("Hood", shooter.getTargetHoodPosition());
    telemetry.addData(
        "Launch Legal",
        sentinel.isLaunchAllowed(follower.getPose()) ? "yes" : "NO - a shot here would not fire");
    telemetry.addData(
        "Shoot Window Here", "%d ms", robot.shotController.shotWindowMsAt(follower.getPose()));

    var solution = robot.shotController.getLastSolution();
    telemetry.addData(
        "Shot Solver",
        String.format(
            Locale.ROOT,
            "%.1f in -> hood %.3f, %.0f rpm [%s]",
            solution.distanceInches(),
            solution.targetHoodPosition(),
            solution.targetRpm(),
            solution.isValid() ? "valid" : solution.validityReason()));
  }

  @Override
  public void stop() {
    robot.shutdown();
  }
}
