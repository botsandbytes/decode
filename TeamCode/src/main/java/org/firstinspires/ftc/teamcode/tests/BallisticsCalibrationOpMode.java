package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

@TeleOp(name = "Ballistics Calibration", group = "Calibration")
@Configurable
public class BallisticsCalibrationOpMode extends LinearOpMode {

  /**
   * One accepted shot: exactly the three numbers the robot commands, plus provenance. No hood angle
   * — the ring's launch angle is not observable on this robot and every attempt to infer it from
   * goal-height shots was unidentifiable, so nothing downstream depends on it any more.
   */
  public record CalibrationPoint(
      double distanceInches, double hoodServoPos, double rpm, String verdict, long timestampMs) {}

  /** Hood step per D-Pad press: up/down is coarse, left/right is fine. */
  private static final double HOOD_STEP_COARSE = 0.050;

  private static final double HOOD_STEP_FINE = 0.005;

  private List<LynxModule> allHubs;
  private Robot robot;
  private FieldManager field;
  private TelemetryManager telemetryM;
  private final List<CalibrationPoint> trials = new ArrayList<>();
  private boolean flywheelRevOn = false;
  private boolean xPressed = false;

  // Starting pose (72, 72, 0) in Pedro Pathing coordinates
  private static final Pose START_POSE = new Pose(72, 72, 0);

  // Legal drive box for the calibration target, in Pedro Pathing field coordinates.
  private static final double MAX_TARGET_X = 85.0;
  private static final double MIN_TARGET_Y = 11.0;

  // Endpoint of the ray the calibration walks out along, away from the blue goal. Aimed straight
  // at the corner (MAX_TARGET_X, MIN_TARGET_Y) on purpose: distance from the goal grows with both
  // +x and -y, so that corner is the single farthest reachable point in the box, and this bearing
  // is therefore the one that maximises calibration range. It reaches 142.1 in, where the previous
  // ray toward (45, 15) left the box at y = 15 after only 123.6 in.
  //
  // 142.1 in is a hard geometric ceiling for this box, not a tuning choice: 144 and 150 in are
  // unreachable here by ANY ray and need the box widened past x = 85 / below y = 11.
  private static final double RAY_END_X = MAX_TARGET_X;
  private static final double RAY_END_Y = MIN_TARGET_Y;

  // Target calibration distances (inches) from the Blue Goal. The top end is 138 rather than the
  // 142.1 ceiling so the far endpoint keeps ~2 in of margin on both bounds, since moveToWaypoint
  // only settles to within 0.8 in and the ceiling sits exactly on the corner.
  private static final double[] CALIBRATION_DISTANCES =
      new double[] {48.0, 72.0, 96.0, 114.0, 126.0, 138.0};

  private final List<Double> skippedDistances = new ArrayList<>();

  /**
   * Distance along the unit ray from the goal at which it first leaves the legal drive box. Each
   * bound contributes a limit only when the ray actually travels toward it.
   */
  private static double maxOnRayDistance(double gx, double gy, double unitX, double unitY) {
    double limit = Double.MAX_VALUE;
    if (unitX > 1e-6) {
      limit = Math.min(limit, (MAX_TARGET_X - gx) / unitX);
    }
    if (unitY < -1e-6) {
      limit = Math.min(limit, (MIN_TARGET_Y - gy) / unitY);
    }
    return limit;
  }

  @Override
  public void runOpMode() throws InterruptedException {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();

    field = PanelsField.INSTANCE.getField();
    if (field != null) {
      field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    // Always calibrate playing Blue Goal derived from config/Field
    MatchProfile profile =
        org.firstinspires.ftc.teamcode.robot.config.generated.config.loadMatchProfile(
            Alliance.BLUE);
    robot = new Robot(hardwareMap, telemetry, profile);
    robot.follower.setStartingPose(START_POSE);

    double blueGoalX = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalX();
    double blueGoalY = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalY();

    telemetry.addLine("Ballistics Human-in-the-Loop Calibration (Blue Alliance)");
    telemetry.addLine("Starting Pose: (72, 72, 0°)");
    telemetry.addData("Target Blue Goal", "(%.1f, %.1f)", blueGoalX, blueGoalY);
    telemetry.addLine("--------------------------------------------------");
    telemetry.addLine("Workflow per Distance Endpoint:");
    telemetry.addLine("  1. Confirmation Prompt prints Coords (X, Y, Heading)");
    telemetry.addLine("  2. Press A to CONFIRM move | Press B to SKIP");
    telemetry.addLine("  3. Autonomous move or manual drive to waypoint");
    telemetry.addLine("  4. D-Pad UP (LONG) / DOWN (SHORT) / A (ACCEPT)");
    telemetry.update();

    waitForStart();

    try {
      for (double distance : CALIBRATION_DISTANCES) {
        if (!opModeIsActive()) break;

        double gx = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalX();
        double gy = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalY();

        double dx = RAY_END_X - gx;
        double dy = RAY_END_Y - gy;
        double distCenter = Math.hypot(dx, dy);
        double unitX = dx / Math.max(1e-6, distCenter);
        double unitY = dy / Math.max(1e-6, distCenter);

        // A distance past where the ray leaves the legal box is simply not calibratable along this
        // bearing, so skip it rather than shoot it anyway. The previous code clamped X and Y
        // independently, which silently slid the target OFF the ray: a requested 144 in became
        // (49.9, 15.0), an actual 124.8 in from the goal on a different bearing, and got logged as
        // a 144 in trial. That is a corrupt calibration point, and it is also why long endpoints
        // looked like the robot "gave up" partway.
        double maxOnRayDistance = maxOnRayDistance(gx, gy, unitX, unitY);
        if (distance > maxOnRayDistance) {
          skippedDistances.add(distance);
          telemetry.clearAll();
          telemetry.addLine("SKIPPING UNREACHABLE ENDPOINT");
          telemetry.addData("Requested distance", "%.1f in", distance);
          telemetry.addData("Max along this ray", "%.1f in", maxOnRayDistance);
          telemetry.addLine("The ray leaves the legal drive box before this distance.");
          telemetry.addLine("Widen RAY_END_X/RAY_END_Y or the bounds to calibrate this far.");
          telemetry.update();
          sleep(1500);
          continue;
        }

        double targetX = gx + distance * unitX;
        double targetY = gy + distance * unitY;
        double targetHeadingRad = Math.atan2(gy - targetY, gx - targetX);
        double targetHeadingDeg = Math.toDegrees(targetHeadingRad);

        Pose targetPose = new Pose(targetX, targetY, targetHeadingRad);

        // --- STEP 1: DRIVER CONFIRMATION PROMPT ---
        boolean confirmed = promptDriverConfirmation(distance, targetPose, targetHeadingDeg);
        if (!confirmed) {
          continue;
        }

        // --- STEP 2: AUTONOMOUS MOVEMENT TO WAYPOINT ---
        moveToWaypoint(targetPose);

        // --- STEP 3: HUMAN-IN-THE-LOOP HOOD TUNING ---
        calibrateEndpoint(distance);
      }

      // Emit the accepted shots as the shot table itself. No fitting: the table IS the model, so
      // the run's output is the same three numbers the robot commands, ready to paste into config.
      List<CalibrationPoint> accepted = new ArrayList<>();
      for (CalibrationPoint p : trials) {
        if ("ACCEPTED".equals(p.verdict())) {
          accepted.add(p);
        }
      }
      accepted.sort((x, y) -> Double.compare(x.distanceInches(), y.distanceInches()));

      String tableYaml = buildTableYaml(accepted);
      saveTrialsToFile(tableYaml);

      telemetry.clearAll();
      telemetry.addLine("Calibration Complete — new shot table");
      telemetry.addLine("========================================");
      for (CalibrationPoint p : accepted) {
        telemetry.addLine(
            String.format("  %.1f, %.4f, %.0f", p.distanceInches(), p.hoodServoPos(), p.rpm()));
      }
      telemetry.addLine("========================================");
      telemetry.addData("Accepted shots", accepted.size());
      if (accepted.size() < 2) {
        telemetry.addLine("WARNING: need 2+ shots to interpolate between.");
      }
      if (!skippedDistances.isEmpty()) {
        StringBuilder skipped = new StringBuilder();
        for (int i = 0; i < skippedDistances.size(); i++) {
          if (i > 0) skipped.append(", ");
          skipped.append(String.format("%.0f", skippedDistances.get(i)));
        }
        telemetry.addData("UNREACHABLE, never shot", "%s in", skipped);
      }
      telemetry.addLine("Paste into config.yaml at shooter.shot_table.points");
      telemetry.addLine("Full copy saved to /sdcard/FIRST/teamcode/shot_table.yaml");
      telemetry.update();

      // Idle until the driver stops the OpMode. The telemetry is re-sent with a running counter
      // every pass: the previous version updated the field drawing but never called
      // telemetry.update() again, so the Driver Station froze on the completion screen and a
      // finished run was indistinguishable from a hang.
      long idleStartMs = System.currentTimeMillis();
      while (opModeIsActive()) {
        for (LynxModule module : allHubs) {
          module.clearBulkCache();
        }
        robot.update();
        drawVisuals();

        telemetry.addLine("Calibration Complete — new shot table");
        telemetry.addLine("========================================");
        for (CalibrationPoint p : accepted) {
          telemetry.addLine(
              String.format("  %.1f, %.4f, %.0f", p.distanceInches(), p.hoodServoPos(), p.rpm()));
        }
        telemetry.addLine("========================================");
        telemetry.addLine("Saved to /sdcard/FIRST/teamcode/shot_table.yaml");
        telemetry.addData(
            "DONE — safe to stop", "idle %ds", (System.currentTimeMillis() - idleStartMs) / 1000);
        telemetry.update();
        sleep(50);
      }
    } finally {
      robot.shutdown();
    }
  }

  private boolean promptDriverConfirmation(double distance, Pose targetPose, double headingDeg) {
    boolean decisionMade = false;
    boolean confirmed = false;

    // Seeded true so a button still held from the previous endpoint has to be released before it
    // can register here. A fixed sleep() debounce could not do this: any hold outlasting it fell
    // straight through, which is how one press of A cascaded into confirming this waypoint and
    // then instantly accepting its hood angle too.
    boolean aPrev = true;
    boolean bPrev = true;

    while (opModeIsActive() && !decisionMade) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }
      robot.update();
      drawVisuals();

      telemetry.addLine("==================================================");
      telemetry.addData("WAYPOINT PROMPT", "Distance = %.1f in", distance);
      telemetry.addData("Calculated Target X", "%.1f in", targetPose.getX());
      telemetry.addData("Calculated Target Y", "%.1f in", targetPose.getY());
      telemetry.addData("Calculated Heading", "%.1f°", headingDeg);
      telemetry.addData(
          "Current Robot Pose",
          "(%.1f, %.1f, %.1f°)",
          robot.follower.getPose().getX(),
          robot.follower.getPose().getY(),
          Math.toDegrees(robot.follower.getPose().getHeading()));
      telemetry.addLine("==================================================");
      telemetry.addLine("Press A (Cross)  -> CONFIRM & Move Robot");
      telemetry.addLine("Press B (Circle) -> SKIP Waypoint");
      telemetry.update();

      if (gamepad1.a && !aPrev) {
        confirmed = true;
        decisionMade = true;
      } else if (gamepad1.b && !bPrev) {
        confirmed = false;
        decisionMade = true;
      }
      aPrev = gamepad1.a;
      bPrev = gamepad1.b;
    }
    return confirmed;
  }

  private void moveToWaypoint(Pose targetPose) {
    robot.follower.holdPoint(targetPose);

    telemetry.clearAll();
    telemetry.addData(
        "Moving to Target Pose",
        "(%.1f, %.1f, %.1f°)",
        targetPose.getX(),
        targetPose.getY(),
        Math.toDegrees(targetPose.getHeading()));
    telemetry.update();

    long startTime = System.currentTimeMillis();
    long timeoutMs = 8000;

    while (opModeIsActive() && (System.currentTimeMillis() - startTime) < timeoutMs) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }
      robot.update();
      drawVisuals();

      Pose currentPose = robot.follower.getPose();
      double distError =
          Math.hypot(
              targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
      double angleErrorDeg =
          Math.toDegrees(
              Math.abs(
                  AngleUnit.normalizeRadians(targetPose.getHeading() - currentPose.getHeading())));

      double gx = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalX();
      double gy = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalY();
      double actualDist = Math.hypot(gy - currentPose.getY(), gx - currentPose.getX());

      telemetry.addData(
          "Moving to Target",
          "(%.1f, %.1f, %.1f°)",
          targetPose.getX(),
          targetPose.getY(),
          Math.toDegrees(targetPose.getHeading()));
      telemetry.addData(
          "Current Robot Pose",
          "(%.1f, %.1f, %.1f°)",
          currentPose.getX(),
          currentPose.getY(),
          Math.toDegrees(currentPose.getHeading()));
      telemetry.addData("Pos Error (in)", "%.2f", distError);
      telemetry.addData("Heading Error (deg)", "%.1f°", angleErrorDeg);
      telemetry.addData("Actual Distance to Goal", "%.2f in", actualDist);
      telemetry.update();

      if ((distError < 0.8 && angleErrorDeg < 1.5)
          || (robot.follower.getVelocity().getMagnitude() < 0.2 && distError < 2.0)) {
        break;
      }
    }
  }

  private void calibrateEndpoint(double targetDistanceInches) {
    var b = org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.ballistics;
    // Ordered travel bounds. min_hood_servo_pos names the servo position at the minimum *angle*,
    // and the linkage is reversed (servo 0.0 is the steepest arc), so it is numerically the larger
    // of the two and cannot be used directly as the low end of the search window.
    double minServo = Math.min(b.min_hood_servo_pos, b.max_hood_servo_pos);
    double maxServo = Math.max(b.min_hood_servo_pos, b.max_hood_servo_pos);

    // Seed from the existing measured shot table, interpolated at this distance. That makes each
    // run start from the last run's answer and refine it, instead of restarting from a model
    // guess every time. When the table is missing or unreadable, fall back to the middle of the
    // hood's travel and the preferred flywheel speed — a known-neutral starting point rather than
    // a value derived from parameters this robot has no way to measure.
    double testPos;
    double pinnedRpm;
    String seedSource;
    try {
      org.firstinspires.ftc.teamcode.ballistics.ShotTable.Shot seed =
          org.firstinspires.ftc.teamcode.ballistics.ShotTable.fromConfig()
              .lookup(targetDistanceInches);
      testPos = seed.hoodServoPosition();
      pinnedRpm = seed.rpm();
      seedSource =
          seed.withinCalibratedRange() ? "shot table" : "shot table (clamped, out of range)";
    } catch (RuntimeException e) {
      testPos = (minServo + maxServo) / 2.0;
      pinnedRpm = b.preferred_shot_rpm;
      seedSource = "defaults (no usable shot table)";
    }
    testPos = Math.clamp(testPos, minServo, maxServo);
    double pinnedPower =
        pinnedRpm / org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.max_rpm;

    double solverInitialRpm = pinnedRpm;
    double seedPos = testPos;
    boolean dpadLeftPrev = false;
    boolean dpadRightPrev = false;
    boolean leftStickPrev = false;
    boolean rightStickPrev = false;
    boolean yPressed = false;

    // All seeded true so a button carried in from the confirmation prompt must be released before
    // it counts. Previously these were level reads behind a sleep(400), so holding A through the
    // drive to the waypoint accepted this endpoint's hood angle the instant the loop opened.
    boolean aPrev = true;
    boolean bPrev = true;
    boolean dpadUpPrev = true;
    boolean dpadDownPrev = true;

    // moveToWaypoint() left the follower in holdPoint mode, which ignores setTeleOpDrive() until
    // manualDrive is switched on. Without this, the sticks below have no effect on the robot.
    robot.follower.startTeleopDrive();

    while (opModeIsActive()) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }

      // Direct hood control: D-Pad up/down steps coarse, left/right steps fine. No bisection and
      // no search window — the driver walks the hood to where the shot scores and accepts it. The
      // old LONG/SHORT bisection assumed which way to move for a given miss, which is only valid
      // on one branch of the trajectory family and drove the hood the wrong way at close range.
      if (gamepad1.dpad_up && !dpadUpPrev) {
        testPos = Math.min(maxServo, testPos + HOOD_STEP_COARSE);
      }
      dpadUpPrev = gamepad1.dpad_up;

      if (gamepad1.dpad_down && !dpadDownPrev) {
        testPos = Math.max(minServo, testPos - HOOD_STEP_COARSE);
      }
      dpadDownPrev = gamepad1.dpad_down;

      if (gamepad1.dpad_right && !dpadRightPrev) {
        testPos = Math.min(maxServo, testPos + HOOD_STEP_FINE);
      }
      dpadRightPrev = gamepad1.dpad_right;

      if (gamepad1.dpad_left && !dpadLeftPrev) {
        testPos = Math.max(minServo, testPos - HOOD_STEP_FINE);
      }
      dpadLeftPrev = gamepad1.dpad_left;

      // HITL RPM Nudge Controls: Left Stick Click (-50 RPM), Right Stick Click (+50 RPM), Button Y
      // (Reset). Pushed to the shooter immediately whenever the flywheel is spinning by either
      // mechanism this OpMode uses — the X-toggle (flywheelRevOn) or a held trigger, which leaves
      // shotController "active" even after release, since nothing here calls stopShot() on its
      // own. Gating on flywheelRevOn alone meant a nudge made while spun up via the trigger updated
      // the on-screen pinnedRpm number but never actually reached the shooter, so the real
      // commanded target stayed wherever it was — this is what "still says 1000" was.
      //
      // Bounded by what the Shooter can physically command, NOT by the solver's
      // [preferred_shot_rpm, max_shot_rpm] policy band. Those bounds are a deliberate runtime
      // choice about where the flywheel should sit; this override exists precisely to test outside
      // them and find out whether that choice is right. Flooring the nudge at preferred_shot_rpm
      // made it impossible to try anything slower than the value being evaluated, which is the
      // whole question at close range. Sub-preferred points fit correctly too —
      // fitParametersWithRpm
      // scales each trial by its own recorded RPM.
      double maxManualRpm =
          org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.max_rpm;
      boolean flywheelSpinning = flywheelRevOn || robot.shotController.isActive();

      if (gamepad1.left_stick_button && !leftStickPrev) {
        pinnedRpm = Math.max(0.0, pinnedRpm - 50.0);
        pinnedPower =
            pinnedRpm
                / org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.max_rpm;
        if (flywheelSpinning) {
          robot.shooter.setTargetPower(pinnedPower);
        }
      }
      leftStickPrev = gamepad1.left_stick_button;

      if (gamepad1.right_stick_button && !rightStickPrev) {
        pinnedRpm = Math.min(maxManualRpm, pinnedRpm + 50.0);
        pinnedPower =
            pinnedRpm
                / org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.max_rpm;
        if (flywheelSpinning) {
          robot.shooter.setTargetPower(pinnedPower);
        }
      }
      rightStickPrev = gamepad1.right_stick_button;

      if (gamepad1.y && !yPressed) {
        pinnedRpm = solverInitialRpm;
        testPos = seedPos;
        pinnedPower =
            pinnedRpm
                / org.firstinspires.ftc.teamcode.robot.config.generated.config.shooter.max_rpm;
        if (flywheelSpinning) {
          robot.shooter.setTargetPower(pinnedPower);
        }
      }
      yPressed = gamepad1.y;

      org.firstinspires.ftc.teamcode.records.BallisticsParameters currentParams =
          org.firstinspires.ftc.teamcode.records.BallisticsParameters.fromConfig();
      double currentAngleDeg =
          org.firstinspires.ftc.teamcode.ballistics.BallisticsModel.servoPosToAngle(
              testPos, currentParams);

      // TeleOp drive & armed heading lock support (runs every tick so Casablanca can rotate
      // chassis)
      double rawTurn = -gamepad1.right_stick_x;
      double forward = -gamepad1.left_stick_y;
      double strafe = -gamepad1.left_stick_x;

      Pose currentPose = robot.follower.getPose();
      double[] adjusted =
          robot.casablanca.adjustDriveInput(
              currentPose, robot.follower.getVelocity(), 0.0, strafe, forward, rawTurn, rawTurn);
      robot.follower.setTeleOpDrive(adjusted[1], adjusted[0], adjusted[2], true);

      // TeleOp & Calibration Shooting Controls. Alignment is still checked (turret aims/gates
      // normally), but useSolvedRpm is forced false: ShotController's own async solver reacts to
      // the robot's live pose, and would otherwise fight pinnedPower with its own answer for
      // whatever the actual distance to goal is right now, defeating the point of pinning this
      // endpoint's calibration shot to a fixed, known RPM.
      if (gamepad1.right_trigger > 0.3) {
        robot.shotController.startShot(pinnedPower, true, false, false);
      } else if (gamepad1.right_bumper) {
        robot.intake.run(
            org.firstinspires.ftc.teamcode.robot.config.generated.config.teleop.intake_power,
            org.firstinspires.ftc.teamcode.robot.config.generated.config.teleop.transfer_power);
      } else if (gamepad1.left_trigger > 0.3 || gamepad1.left_bumper) {
        robot.intake.stop();
        robot.shotController.stopShot();
      }

      if (gamepad1.x && !xPressed) {
        flywheelRevOn = !flywheelRevOn;
        robot.shooter.setTargetPower(flywheelRevOn ? pinnedPower : 0.0);
      }
      xPressed = gamepad1.x;

      robot.shooter.setTargetHoodPosition(testPos);
      robot.update();
      drawVisuals();

      double gx = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalX();
      double gy = org.firstinspires.ftc.teamcode.records.Field.getBlueGoalY();
      double actualDist = Math.hypot(gy - currentPose.getY(), gx - currentPose.getX());

      // Flywheel gate diagnostics
      double flywheelVelocity = Math.abs(robot.shotController.getFlywheelVelocity());
      double flywheelTarget = Math.abs(robot.shotController.getFlywheelTarget());
      double minThreshold =
          Math.abs(
              flywheelTarget
                  * org.firstinspires
                      .ftc
                      .teamcode
                      .robot
                      .config
                      .generated
                      .config
                      .shooter
                      .min_transfer_threshold);
      boolean flywheelGateMet = robot.shotController.isFlywheelReady();

      String gateStatus = flywheelGateMet ? "PASS" : "FAIL (not feeding)";

      telemetry.addData("Target Distance (in)", targetDistanceInches);
      telemetry.addData("Actual Distance to Goal", "%.2f in", actualDist);
      telemetry.addData(
          "Current Robot Pose",
          "(%.1f, %.1f, %.1f°)",
          robot.follower.getPose().getX(),
          robot.follower.getPose().getY(),
          Math.toDegrees(robot.follower.getPose().getHeading()));
      double rpmDelta = pinnedRpm - solverInitialRpm;
      String rpmDisplay =
          Math.abs(rpmDelta) < 1e-3
              ? String.format("%.0f RPM (Solver)", pinnedRpm)
              : String.format("%.0f RPM (HITL %+.0f)", pinnedRpm, rpmDelta);

      telemetry.addLine("--- Hood & RPM ---");
      telemetry.addData("Hood Servo Pos", "%.4f", testPos);
      telemetry.addData("Shooter target hood", "%.4f", robot.shooter.getTargetHoodPosition());
      telemetry.addData("Servo readback", "%.4f", robot.shooter.getHoodPositionReadback());
      telemetry.addData(
          "Hood travel", "[%.3f, %.3f]  seed %.4f (%s)", minServo, maxServo, seedPos, seedSource);
      telemetry.addData("Pinned RPM Setpoint", rpmDisplay);
      telemetry.addData(
          "Nudge Range", "0 - %.0f  (solver picked %.0f)", maxManualRpm, solverInitialRpm);
      if (pinnedRpm < b.preferred_shot_rpm) {
        telemetry.addData(
            "NOTE",
            "below preferred_shot_rpm %.0f — accept here and it gets recommended at the end",
            b.preferred_shot_rpm);
      }
      telemetry.addData(
          "Flywheel Rev", flywheelRevOn ? String.format("ON (%s)", rpmDisplay) : "OFF");
      telemetry.addLine("--- Shooter Gate ---");
      telemetry.addData("Flywheel Velocity", "%.0f ticks/s", flywheelVelocity);
      telemetry.addData("Flywheel Target", "%.0f ticks/s", flywheelTarget);
      telemetry.addData(
          "Min Threshold",
          "%.0f ticks/s (%.0f%%)",
          minThreshold,
          org.firstinspires
                  .ftc
                  .teamcode
                  .robot
                  .config
                  .generated
                  .config
                  .shooter
                  .min_transfer_threshold
              * 100);
      telemetry.addData("Gate 1 (Flywheel)", gateStatus);
      telemetry.addData(
          "Gate 3 (Solver)",
          "BYPASSED for calibration — %s",
          robot.shotController.getLastSolution().validityReason());
      telemetry.addLine("--- Control Loop ---");
      telemetry.addData("RAW signed velocity", "%.0f ticks/s", robot.shooter.getShooterVelocity());
      telemetry.addData("Setpoint", "%.0f ticks/s", robot.shooter.getLastTargetVelocity());
      telemetry.addData("Error", "%.0f ticks/s", robot.shooter.getLastError());
      telemetry.addData("Feedforward term", "%.3f", robot.shooter.getLastFeedforwardTerm());
      telemetry.addData("PID term", "%.3f", robot.shooter.getLastPidTerm());
      telemetry.addData("Integral term", "%.3f", robot.shooter.getLastIntegralTerm());
      telemetry.addData("Motor command", "%.3f", robot.shooter.getLastCommand());
      telemetry.addLine("---------------------------------------------");
      telemetry.addLine("Controls:");
      telemetry.addLine("  Sticks          : Drive Robot (TeleOp Manual Drive)");
      telemetry.addLine("  Right Trigger   : TeleOp Aim & Auto-Shoot");
      telemetry.addLine("  Right Bumper    : Manual Intake & Feed");
      telemetry.addLine("  Left Bumper/LT  : Stop Intake & Shot");
      telemetry.addLine("  Button X        : Toggle Flywheel Rev");
      telemetry.addLine(
          String.format("  DPad Up/Down    : Hood COARSE -/+ %.3f servo", HOOD_STEP_COARSE));
      telemetry.addLine(
          String.format("  DPad Left/Right : Hood FINE   -/+ %.3f servo", HOOD_STEP_FINE));
      telemetry.addLine("  Stick Clicks L/R: Nudge RPM -/+ 50 (full 0-max range)");
      telemetry.addLine("  Button Y        : Reset hood + RPM to seed");
      telemetry.addLine("  Button A        : ACCEPT current hood + RPM");
      telemetry.addLine("  Button B        : SKIP Endpoint");
      telemetry.update();

      // Panels telemetry — shooter gate status
      if (telemetryM != null) {
        telemetryM.addLine("=== BALLISTICS CALIBRATION ===");
        telemetryM.addData("Distance Target (in)", String.format("%.1f", targetDistanceInches));
        telemetryM.addData("Actual Distance (in)", String.format("%.2f", actualDist));
        telemetryM.addData("Hood Servo Pos", String.format("%.4f", testPos));
        telemetryM.addData("Pinned RPM", String.format("%.0f", pinnedRpm));
        telemetryM.addLine("--- Shooter Gate ---");
        telemetryM.addData("Flywheel Velocity (ticks/s)", String.format("%.0f", flywheelVelocity));
        telemetryM.addData("Flywheel Target (ticks/s)", String.format("%.0f", flywheelTarget));
        telemetryM.addData("Min Threshold (ticks/s)", String.format("%.0f", minThreshold));
        telemetryM.addData("Gate 1 (Flywheel)", gateStatus);
        telemetryM.addLine("--- Control Loop ---");
        telemetryM.addData(
            "Setpoint (ticks/s)", String.format("%.0f", robot.shooter.getLastTargetVelocity()));
        telemetryM.addData("Error (ticks/s)", String.format("%.0f", robot.shooter.getLastError()));
        telemetryM.addData(
            "Feedforward term", String.format("%.3f", robot.shooter.getLastFeedforwardTerm()));
        telemetryM.addData("PID term", String.format("%.3f", robot.shooter.getLastPidTerm()));
        telemetryM.addData("Motor command", String.format("%.3f", robot.shooter.getLastCommand()));
        telemetryM.update();
      }

      // The endpoint ends only when the driver says so. There is no automatic accept: the old
      // bisection ended the endpoint on its own once its window narrowed, which turned a single
      // mis-pressed D-Pad direction into a silently recorded, never-verified shot.
      boolean aPressed = gamepad1.a && !aPrev;
      boolean bPressed = gamepad1.b && !bPrev;
      aPrev = gamepad1.a;
      bPrev = gamepad1.b;

      if (aPressed) {
        trials.add(
            new CalibrationPoint(
                actualDist, testPos, pinnedRpm, "ACCEPTED", System.currentTimeMillis()));
        saveProgress();
        break;
      } else if (bPressed) {
        trials.add(
            new CalibrationPoint(
                actualDist, testPos, pinnedRpm, "SKIPPED", System.currentTimeMillis()));
        saveProgress();
        break;
      }
    }

    robot.shotController.stopShot();
    robot.intake.stop();
    robot.shooter.setTargetPower(0.0);
    flywheelRevOn = false;
  }

  private void drawVisuals() {
    if (field != null) {
      DrawingUtil.drawCasablancaZones(field, robot.sentinel);
      OpModeUtil.drawRobot(field, robot.follower, robot.turret, 0.0, 144.0);
    }
  }

  /**
   * Flushes everything accepted so far to disk after each endpoint.
   *
   * <p>The full table was previously only written once the whole run finished, so stopping the
   * OpMode part way — a dead battery, a hardware fault, a mis-press — silently discarded every shot
   * taken up to that point. The trials list lives only in memory, and an OpMode that never reaches
   * its last line never saves.
   */
  private void saveProgress() {
    List<CalibrationPoint> accepted = new ArrayList<>();
    for (CalibrationPoint p : trials) {
      if ("ACCEPTED".equals(p.verdict())) {
        accepted.add(p);
      }
    }
    accepted.sort((x, y) -> Double.compare(x.distanceInches(), y.distanceInches()));
    saveTrialsToFile(buildTableYaml(accepted));
  }

  /** Renders the accepted shots as a paste-ready shooter.shot_table.points block. */
  private String buildTableYaml(List<CalibrationPoint> accepted) {
    StringBuilder sb = new StringBuilder();
    sb.append("  shot_table:\n");
    sb.append("    # distance_in, hood_servo, flywheel_rpm\n");
    sb.append("    points: [\n");
    for (int i = 0; i < accepted.size(); i++) {
      CalibrationPoint p = accepted.get(i);
      sb.append(
          String.format(
              "        %.1f, %.4f, %.0f%s%n",
              p.distanceInches(),
              p.hoodServoPos(),
              p.rpm(),
              (i == accepted.size() - 1) ? "" : ","));
    }
    sb.append("      ]\n");
    return sb.toString();
  }

  /**
   * Writes the paste-ready table plus every trial, accepted or not. The file is truncated on each
   * run, so pull it before re-running if the previous run's shots still matter.
   */
  private void saveTrialsToFile(String tableYaml) {
    File dir = new File("/sdcard/FIRST/teamcode/");
    if (!dir.exists()) {
      dir.mkdirs();
    }
    try (FileWriter writer = new FileWriter(new File(dir, "shot_table.yaml"), false)) {
      writer.write("# Generated by Ballistics Calibration\n");
      writer.write("# Paste the block below into config.yaml under shooter:\n\n");
      writer.write(tableYaml);
      writer.write("\n# --- every trial this run ---\n");
      for (CalibrationPoint p : trials) {
        writer.write(
            String.format(
                "# %.2f in, servo %.4f, rpm %.0f, %s, t=%d%n",
                p.distanceInches(), p.hoodServoPos(), p.rpm(), p.verdict(), p.timestampMs()));
      }
    } catch (IOException e) {
      telemetry.addData("Error", "Failed to write shot table file: " + e.getMessage());
    }
  }
}
