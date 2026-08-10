package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Odometry never starts the match knowing where it is; this OpMode gives a human the delta needed
 * to physically carry the robot from field center (72, 72, 0) to wherever the selected auto's
 * config start pose actually is, then lets that arrival be confirmed as the new origin.
 *
 * <p>Gamepad1: left stick translates and right stick turns (robot-centric, scaled down for fine
 * positioning), dpad left/right selects alliance, dpad up/down selects Near ({@code
 * auto_poses.normal}) vs Far ({@code auto_poses.opposite}), A confirms the robot is now sitting at
 * the selected start pose and re-zeros the follower there.
 */
@TeleOp(name = "Start Pose Finder", group = "Calibration")
@Configurable
public class StartPoseFinderOpMode extends OpMode {

  /**
   * Ceiling on stick-commanded power. This OpMode exists to walk the robot the last few inches onto
   * a start pose, so it is deliberately slower than match teleop — {@code config.teleop.max_speed}
   * would overshoot the target by more than the delta being nulled out.
   */
  public static double driveSpeed = 0.35;

  private Follower follower;

  private Alliance alliance = Alliance.RED;
  private boolean near = true;
  private Pose configStartPose = new Pose(0, 0, 0);

  private boolean lastDpadLeft, lastDpadRight, lastDpadUp, lastDpadDown, lastA;

  @Override
  public void init() {
    config.reload();
    follower = Constants.createCachedFollower(hardwareMap);
    follower.setStartingPose(new Pose(72, 72, 0));
    recomputeConfigStartPose();
  }

  @Override
  public void init_loop() {
    handleSelectionInput();
    telemetry.addLine("== Start Pose Finder ==");
    telemetry.addData("Alliance (dpad left/right)", alliance);
    telemetry.addData("Auto (dpad up/down)", near ? "NEAR (normal)" : "FAR (opposite)");
    telemetry.addData(
        "Config start pose",
        "(%.1f, %.1f, %.1f deg)",
        configStartPose.getX(),
        configStartPose.getY(),
        Math.toDegrees(configStartPose.getHeading()));
    telemetry.update();
  }

  @Override
  public void start() {
    follower.startTeleopDrive();
  }

  @Override
  public void loop() {
    handleSelectionInput();

    // Written before update() so the vector lands on the motors this loop rather than next.
    // Deliberately raw and robot-centric, bypassing Casablanca: the whole premise of this OpMode is
    // that the follower's pose is not yet trusted, and Casablanca's zone protections are computed
    // from that pose. Cubed sticks give resolution near center, which is where all the work is.
    follower.setTeleOpDrive(
        Math.clamp(-Math.pow(gamepad1.left_stick_y, 3), -driveSpeed, driveSpeed),
        Math.clamp(-Math.pow(gamepad1.left_stick_x, 3), -driveSpeed, driveSpeed),
        Math.clamp(-Math.pow(gamepad1.right_stick_x, 3), -driveSpeed, driveSpeed),
        true);

    follower.update();

    Pose currentPose = follower.getPose();
    double dx = configStartPose.getX() - currentPose.getX();
    double dy = configStartPose.getY() - currentPose.getY();
    double distance = currentPose.distanceFrom(configStartPose);
    double bearingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)));

    telemetry.addLine("== Start Pose Finder ==");
    telemetry.addData("Alliance (dpad left/right)", alliance);
    telemetry.addData("Auto (dpad up/down)", near ? "NEAR (normal)" : "FAR (opposite)");
    telemetry.addLine();
    telemetry.addData(
        "Raw follower pose",
        "(%.2f, %.2f, %.1f deg)",
        currentPose.getX(),
        currentPose.getY(),
        Math.toDegrees(currentPose.getHeading()));
    telemetry.addData(
        "Raw start pose",
        "(%.2f, %.2f, %.1f deg)",
        configStartPose.getX(),
        configStartPose.getY(),
        Math.toDegrees(configStartPose.getHeading()));
    telemetry.addLine();
    telemetry.addData("Distance to start", "%.2f in", distance);
    telemetry.addData("Move", "%+.2f in X, %+.2f in Y", dx, dy);
    telemetry.addData("Bearing (field frame)", "%.1f deg", bearingDeg);
    telemetry.addLine();
    telemetry.addData("A", "confirm robot is at start pose -> re-zero follower here");
    telemetry.update();

    if (gamepad1.a && !lastA) {
      follower.setStartingPose(configStartPose);
    }
    lastA = gamepad1.a;
  }

  private void handleSelectionInput() {
    if (gamepad1.dpad_left && !lastDpadLeft) {
      alliance = Alliance.RED;
      recomputeConfigStartPose();
    }
    if (gamepad1.dpad_right && !lastDpadRight) {
      alliance = Alliance.BLUE;
      recomputeConfigStartPose();
    }
    if (gamepad1.dpad_up && !lastDpadUp) {
      near = true;
      recomputeConfigStartPose();
    }
    if (gamepad1.dpad_down && !lastDpadDown) {
      near = false;
      recomputeConfigStartPose();
    }
    lastDpadLeft = gamepad1.dpad_left;
    lastDpadRight = gamepad1.dpad_right;
    lastDpadUp = gamepad1.dpad_up;
    lastDpadDown = gamepad1.dpad_down;
  }

  private void recomputeConfigStartPose() {
    String allianceStr = alliance == Alliance.RED ? "red" : "blue";
    configStartPose =
        near
            ? ConfigLoader.loadMerged(
                    config.NormalAuto.class, "auto_poses.normal." + allianceStr, "auto")
                .start
            : ConfigLoader.loadMerged(
                    config.OppositeAuto.class, "auto_poses.opposite." + allianceStr, "auto")
                .start;
  }
}
