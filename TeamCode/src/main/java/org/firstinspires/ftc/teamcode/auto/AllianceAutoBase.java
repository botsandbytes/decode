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
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
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
    robot.update();

    OpModeUtil.savePose(alliance, follower.getPose());
    OpModeUtil.drawRobot(field, follower, turret, goalX, goalY);

    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    addShooterDiagnostics();
    telemetry.update();
  }

  /**
   * Flywheel and shot-solver state, every loop.
   *
   * <p>ShotController only publishes its gate telemetry while a shot is armed, which in auto is
   * about a second per scoring cycle — so for most of a run the driver station showed nothing but
   * the pose, and a flywheel that was failing to reach its setpoint looked identical to one that
   * was fine. These lines are the difference between "auto did not shoot" and knowing which of
   * spin-up, aim, or the solver is responsible.
   *
   * <p>"Launch Legal" is the fourth way a shot can fail and the only silent one: the shot command
   * ends itself the instant the robot is outside every launch zone, so a score pose that lands a
   * few inches short does not shoot badly, it does not shoot at all. Watch this line at each score
   * pose before blaming the flywheel.
   */
  private void addShooterDiagnostics() {
    double target = shooter.getLastTargetVelocity();
    double actual = Math.abs(shooter.getShooterVelocity());
    telemetry.addData(
        "Flywheel",
        String.format(
            "%.0f / %.0f ticks/s (%.2fx)", actual, target, target > 0 ? actual / target : 0.0));
    telemetry.addData(
        "Flywheel Cmd",
        String.format(
            "%.3f  (pid %.3f + i %.3f + ff %.3f) x%.2f",
            shooter.getLastCommand(),
            shooter.getLastPidTerm(),
            shooter.getLastIntegralTerm(),
            shooter.getLastFeedforwardTerm(),
            shooter.getLastVoltageScale()));
    telemetry.addData("Bus Voltage", String.format("%.2f V", shooter.getLastBusVoltage()));
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
