package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.Field;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

@TeleOp(name = "Casablanca Test", group = "Test")
public class CasablancaTest extends OpMode {

  private Follower follower;
  private FieldManager field;
  private List<LynxModule> allHubs;
  private Turret turret;
  private Sentinel sentinel;
  private Casablanca casablanca;

  private double goalX;
  private double goalY;

  private Pose startPose = new Pose(72, 72, 0);

  @Override
  public void init() {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    // Initialize Field Manager
    field = PanelsField.INSTANCE.getField();
    field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

    // Initialize Hardware (Follower)
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    List<DcMotorEx> motors = hardwareMap.getAll(DcMotorEx.class);
    for (DcMotorEx motor : motors) {
      motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startPose);

    goalX = Field.getRedGoalX();
    goalY = Field.getRedGoalY();

    sentinel = new Sentinel(Alliance.RED);
    casablanca = new Casablanca(sentinel);
    turret = new Turret(hardwareMap, telemetry, follower);
  }

  @Override
  public void start() {
    follower.startTeleopDrive();
  }

  @Override
  public void loop() {
    // Clear bulk cache
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }

    follower.update();

    handleDrive();
    drawField();

    telemetry.addData("X", follower.getPose().getX());
    telemetry.addData("Y", follower.getPose().getY());
    telemetry.addData("Heading", follower.getPose().getHeading());
    telemetry.update();
  }

  private void handleDrive() {
    double forward = -gamepad1.left_stick_y;
    double strafe = -gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;

    // Convert Pedro Pose to FTC Pose2D
    Pose pedroPose = follower.getPose();

    // Adjust inputs using Casablanca
    // Passing 0 for current velocity as it seems unused or we don't have it easily
    double[] adjusted =
        casablanca.adjustDriveInput(
            follower.getPose(), follower.getVelocity(), strafe, forward, turn);

    // Casablanca returns [strafe, forward, turn]
    // Follower expects [forward, strafe, turn]
    follower.setTeleOpDrive(adjusted[1], adjusted[0], adjusted[2], false);

    telemetry.addData(
        "Pos", "X:%.1f Y:%.1f H:%.2f", pedroPose.getX(), pedroPose.getY(), pedroPose.getHeading());
    telemetry.addData("Input", "F:%.2f S:%.2f T:%.2f", forward, strafe, turn);
    telemetry.addData("Output", "F:%.2f S:%.2f T:%.2f", adjusted[1], adjusted[0], adjusted[2]);

    // Debug info
    if (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1) {
      double inputMag = Math.hypot(forward, strafe);
      double outputMag = Math.hypot(adjusted[1], adjusted[0]);
      telemetry.addData("Magnitude", "In:%.2f Out:%.2f", inputMag, outputMag);
      if (inputMag > 0.1 && outputMag < 0.05) {
        telemetry.addData("WARNING", "Movement blocked!");
      }
    }
  }

  private void drawField() {
    if (field != null) {
      DrawingUtil.drawCasablancaZones(field, sentinel);
      DrawingUtil.drawRobotOnField(
          field,
          follower.getPose().getX(),
          follower.getPose().getY(),
          follower.getPose().getHeading(),
          Math.toRadians(turret.getCurrentTurnAngle()),
          goalX,
          goalY);
    }
  }
}
