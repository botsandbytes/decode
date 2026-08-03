package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.field.FieldManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.LaunchParameters;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/**
 * Generic, season-agnostic TeleOp base class. Encapsulates robot initialization, blackboard pose
 * persistence, gamepad binding, telemetry, field drawing, vision updates, and default teleop drive.
 */
public abstract class TeleOpBase extends OpMode {

  public static class ButtonTracker {
    private boolean currentState = false;
    private boolean previousState = false;

    public void update(boolean state) {
      previousState = currentState;
      currentState = state;
    }

    public boolean onTrue() {
      return currentState && !previousState;
    }

    public boolean onFalse() {
      return !currentState && previousState;
    }

    public boolean state() {
      return currentState;
    }
  }

  protected Robot robot;
  protected Follower follower;
  protected Intake intake;
  protected Shooter shooter;
  protected Turret turret;
  protected ShotController shotController;
  protected Sentinel sentinel;
  protected Casablanca casablanca;
  protected MatchProfile profile;

  protected Gamepad driver;
  protected Gamepad operator;

  protected ButtonTracker operatorA = new ButtonTracker();
  protected ButtonTracker operatorB = new ButtonTracker();
  protected ButtonTracker operatorX = new ButtonTracker();
  protected ButtonTracker operatorY = new ButtonTracker();
  protected ButtonTracker operatorDpadUp = new ButtonTracker();
  protected ButtonTracker operatorDpadDown = new ButtonTracker();
  protected ButtonTracker operatorDpadRight = new ButtonTracker();
  protected ButtonTracker operatorRightTrigger = new ButtonTracker();
  protected ButtonTracker operatorLeftTrigger = new ButtonTracker();

  protected ButtonTracker driverDpadLeft = new ButtonTracker();
  protected ButtonTracker driverDpadRight = new ButtonTracker();
  protected ButtonTracker driverRightTrigger = new ButtonTracker();
  protected ButtonTracker driverLeftTrigger = new ButtonTracker();

  protected FieldManager field;
  protected TelemetryManager telemetryM;
  protected LaunchParameters currentParams;

  protected Command teleOpDriveCommand;

  protected abstract void buildCommands();

  protected abstract void onLoop();

  @Override
  public void init() {
    config.reload();
    Alliance alliance = (Alliance) blackboard.get("ALLIANCE");
    if (alliance == null) {
      alliance = Alliance.RED;
    }
    profile = config.loadMatchProfile(alliance);

    robot = new Robot(hardwareMap, telemetry, profile);
    follower = robot.follower;
    intake = robot.intake;
    shooter = robot.shooter;
    turret = robot.turret;
    shotController = robot.shotController;
    sentinel = robot.sentinel;
    casablanca = robot.casablanca;

    field = OpModeUtil.initPanelsField();
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    Pose savedPose = OpModeUtil.getSavedPose(alliance, profile.startPose());
    follower.setStartingPose(savedPose);

    robot.vision.initAprilTag(hardwareMap, true);
    casablanca.reset();

    driver = gamepad1;
    operator = gamepad2;

    shooter.setShooterPIDFCoefficients();

    double maxSpeed = config.teleop.max_speed;
    teleOpDriveCommand =
        Command.build()
            .setExecute(
                () -> {
                  double y = Math.clamp(-Math.pow(driver.left_stick_y, 3), -maxSpeed, maxSpeed);
                  double x = Math.clamp(-Math.pow(driver.left_stick_x, 3), -maxSpeed, maxSpeed);
                  double r = Math.clamp(-Math.pow(driver.right_stick_x, 3), -maxSpeed, maxSpeed);

                  double[] adjusted =
                      casablanca.adjustDriveInput(
                          follower.getPose(),
                          follower.getVelocity(),
                          follower.getAngularVelocity(),
                          x,
                          y,
                          r,
                          -driver.right_stick_x);
                  follower.setTeleOpDrive(adjusted[1], adjusted[0], adjusted[2], false);
                })
            .requiring(follower);

    buildCommands();
  }

  @Override
  public void start() {
    follower.startTeleopDrive();
    OpModeUtil.setupTurretAndShooter(turret, shooter);
    teleOpDriveCommand.schedule();
  }

  private void updateGamepads() {
    operatorA.update(operator.a);
    operatorB.update(operator.b);
    operatorX.update(operator.x);
    operatorY.update(operator.y);
    operatorDpadUp.update(operator.dpad_up);
    operatorDpadDown.update(operator.dpad_down);
    operatorDpadRight.update(operator.dpad_right);
    operatorRightTrigger.update(operator.right_trigger > 0.5);
    operatorLeftTrigger.update(operator.left_trigger > 0.5);

    driverDpadLeft.update(driver.dpad_left);
    driverDpadRight.update(driver.dpad_right);
    driverRightTrigger.update(driver.right_trigger > 0.5);
    driverLeftTrigger.update(driver.left_trigger > 0.5);
  }

  @Override
  public void loop() {
    updateGamepads();

    currentParams =
        shooter.calculateLaunchParameters(
            follower.getPose(), profile.goalX(), profile.goalY(), profile.alliance());

    robot.update();

    if (!Scheduler.isScheduled(teleOpDriveCommand) && !hasActiveDriveOverride()) {
      teleOpDriveCommand.schedule();
    }

    onLoop();

    handleVision();

    OpModeUtil.drawRobot(field, follower, turret, profile.goalX(), profile.goalY());
    DrawingUtil.drawCasablancaZones(field, sentinel);

    OpModeUtil.savePose(profile.alliance(), follower.getPose());

    telemetryM.addData("Alliance", profile.alliance());
    telemetryM.addData("Pose", follower.getPose());
    telemetryM.addData("Turret Angle", turret.getCurrentTurnAngle());
    telemetryM.addData("Is Shooting", shotController.isActive());
    telemetryM.update();
  }

  protected boolean hasActiveDriveOverride() {
    return false;
  }

  private void handleVision() {
    if (operatorDpadDown.onTrue()) {
      robot.vision.resumeStreaming();
    }

    if (operator.dpad_down) {
      Pose visionPose = robot.vision.updateAprilTagPose();
      if (robot.vision.isTagFound() && visionPose != null) {
        follower.setPose(visionPose);
        telemetryM.addLine(
            "Pose updated: X="
                + String.format("%.2f", visionPose.getX())
                + " Y="
                + String.format("%.2f", visionPose.getY())
                + " H="
                + String.format("%.2f", Math.toDegrees(visionPose.getHeading())));
        robot.vision.stopStreaming();
      } else {
        telemetryM.addData("Vision [On-Demand]", "Searching for Tag...");
      }
    }

    if (operatorDpadDown.onFalse()) {
      robot.vision.stopStreaming();
    }
  }
}
