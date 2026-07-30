package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Configurable
@TeleOp(name = "Manual Turret Turn Tester", group = "Test")
public class ManualTurretTurnTest extends LinearOpMode {
  private Turret turret;
  private Follower follower;
  public static int turn_degrees = 60;

  @Override
  public void runOpMode() throws InterruptedException {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(0, 0, 0));
    turret = new Turret(hardwareMap, telemetry, follower);

    waitForStart();

    while (opModeIsActive()) {
      follower.update();

      if (gamepad1.dpad_down) {
        turret.setInitialHeading(0);
      } else if (gamepad1.dpad_right) {
        turret.setTargetTurnAngle(turn_degrees);
      }

      turret.updateTurret(follower.getPose());

      telemetry.addData("Turret Angle", turret.getCurrentTurnAngle());
      telemetry.addData("Target Angle", turret.getTargetTurnAngle());
      telemetry.update();
    }
  }
}
