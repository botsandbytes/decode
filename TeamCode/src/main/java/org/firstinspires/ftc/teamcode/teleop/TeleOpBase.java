package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.field.FieldManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.LaunchParameters;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/**
 * Generic, season-agnostic TeleOp base class. Encapsulates robot initialization, blackboard pose
 * persistence, gamepad binding, telemetry, field drawing, vision updates, and default teleop drive.
 */
public abstract class TeleOpBase extends OpMode {

  protected Robot robot;
  protected Follower follower;
  protected Intake intake;
  protected Shooter shooter;
  protected Turret turret;
  protected ShotController shotController;
  protected Sentinel sentinel;
  protected Casablanca casablanca;
  protected MatchProfile profile;

  protected SDKGamepad driver;
  protected SDKGamepad operator;

  protected EnhancedBooleanSupplier operatorRightTrigger;
  protected EnhancedBooleanSupplier operatorLeftTrigger;
  protected EnhancedBooleanSupplier driverRightTrigger;
  protected EnhancedBooleanSupplier driverLeftTrigger;

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
    turret.setInitialHeading(follower.getHeading());

    robot.vision.initAprilTag(hardwareMap, true);
    casablanca.reset();

    Pasteurized.gamepad1(new SDKGamepad(gamepad1));
    Pasteurized.gamepad2(new SDKGamepad(gamepad2));
    driver = (SDKGamepad) Pasteurized.gamepad1();
    operator = (SDKGamepad) Pasteurized.gamepad2();

    operatorRightTrigger = operator.rightTrigger().conditionalBindState().greaterThan(0.5).bind();
    operatorLeftTrigger = operator.leftTrigger().conditionalBindState().greaterThan(0.5).bind();
    driverRightTrigger = driver.rightTrigger().conditionalBindState().greaterThan(0.5).bind();
    driverLeftTrigger = driver.leftTrigger().conditionalBindState().greaterThan(0.5).bind();

    shooter.setShooterPIDFCoefficients();

    double maxSpeed = config.teleop.max_speed;
    teleOpDriveCommand =
        Command.build()
            .setExecute(
                () -> {
                  double y =
                      Math.clamp(-Math.pow(driver.leftStickY().state(), 3), -maxSpeed, maxSpeed);
                  double x =
                      Math.clamp(-Math.pow(driver.leftStickX().state(), 3), -maxSpeed, maxSpeed);
                  double r =
                      Math.clamp(-Math.pow(driver.rightStickX().state(), 3), -maxSpeed, maxSpeed);

                  double[] adjusted =
                      casablanca.adjustDriveInput(
                          follower.getPose(), follower.getVelocity(), x, y, r);
                  follower.setTeleOpDrive(adjusted[1], adjusted[0], adjusted[2], false);
                })
            .requiring(follower);

    buildCommands();
  }

  @Override
  public void start() {
    follower.startTeleopDrive();
    OpModeUtil.setupTurretAndShooter(turret, shooter, follower.getHeading());
    teleOpDriveCommand.schedule();
  }

  @Override
  public void loop() {
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
    if (operator.dpadDown().state()) {
      Pose visionPose = robot.vision.updateAprilTagPose();
      if (robot.vision.isTagFound()) {
        follower.setPose(visionPose);
        telemetryM.addLine(
            "Pose updated: X="
                + visionPose.getX()
                + " Y="
                + visionPose.getY()
                + " H="
                + visionPose.getHeading());
      } else {
        telemetryM.addData("Vision", "No Tag Found");
      }
      robot.vision.stopStreaming();
    }
  }
}
