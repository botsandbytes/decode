package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ballistics.ShotSolver;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.ballistics.ShotTimeTable;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.Field;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

public class ShotController {
  private final Shooter shooter;
  private final Turret turret;
  private final Intake intake;
  private final Supplier<Pose> poseSupplier;
  private final Supplier<Pose> velocitySupplier;
  private final Casablanca casablanca;
  private final Alliance alliance;
  private final Telemetry telemetry;

  private boolean active = false;
  private boolean checkAlignment = false;
  private boolean useSolvedRpm = false;
  private boolean requireValidSolution = true;

  /** Latching state of the feed gate; see {@link #updateFeedGate()}. */
  private boolean feeding = false;

  /** Whether the last {@link #periodic()} actually ran the intake, i.e. all three gates agreed. */
  private boolean feedCommanded = false;

  // Live ball counting, from the flywheel dip each one costs. Built fresh per shot rather than
  // reused so a hot-pushed config.yaml threshold takes effect on the very next shot, the same as
  // every other config read in this class.
  private FlywheelDipDetector ballDetector = newBallDetector();
  private boolean ballDetectionArmed = false;
  private int ballsFired = 0;

  /** Flywheel setpoint most recently commanded from a solution; NaN until the first one lands. */
  private double commandedSolvedRpm = Double.NaN;

  private final ElapsedTime timer = new ElapsedTime();

  // Solver thread fields
  private final Object solveLock = new Object();
  private ShotInputs pendingInputs;
  private double lastFlightTime = 0.4;
  private volatile ShotSolution lastSolution =
      new ShotSolution(0.15, 0.0, 0.0, 0.0, 0.4, 0.0, 72.0, false, "Not initialized");
  private volatile boolean solving = false;
  private volatile long lastSolveDurationMs = 0;
  private volatile boolean running = true;
  private final Thread solverThread;

  public ShotController(
      Shooter shooter,
      Turret turret,
      Intake intake,
      Supplier<Pose> poseSupplier,
      Supplier<Pose> velocitySupplier,
      Casablanca casablanca,
      Alliance alliance,
      Telemetry telemetry) {
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
    this.casablanca = casablanca;
    this.alliance = alliance;
    this.telemetry = telemetry;

    solverThread = new Thread(this::solveLoop, "ShotSolver");
    solverThread.setDaemon(true);
    solverThread.start();
  }

  public void startShot(double power, boolean checkAlignment) {
    startShot(power, checkAlignment, checkAlignment);
  }

  public void startShot(double power, boolean checkAlignment, boolean useSolvedRpm) {
    startShot(power, checkAlignment, useSolvedRpm, true);
  }

  public void startShot(
      double power, boolean checkAlignment, boolean useSolvedRpm, boolean requireValidSolution) {
    this.active = true;
    this.checkAlignment = checkAlignment;
    this.useSolvedRpm = useSolvedRpm;
    this.requireValidSolution = requireValidSolution;
    this.commandedSolvedRpm = Double.NaN;
    this.feeding = false;
    this.timer.reset();
    this.ballDetector = newBallDetector();
    this.ballDetectionArmed = false;
    this.ballsFired = 0;
    shooter.setTargetPower(power);
    if (checkAlignment && turret != null && turret.isEnabled()) {
      turret.setAimMode(Turret.AimMode.AIM_AT_GOAL);
    }
  }

  public void stopShot() {
    this.active = false;
    shooter.setTargetPower(0.0);
    intake.stop();
    turret.setTargetAzimuth(Double.NaN);
    if (casablanca != null) {
      casablanca.setArmedAimTarget(0.0, false);
    }
    if (checkAlignment && turret != null && turret.isEnabled()) {
      turret.setHoldAngle(0.0);
      turret.setAimMode(Turret.AimMode.HOLD);
    }
    this.checkAlignment = false;
    this.useSolvedRpm = false;
    this.commandedSolvedRpm = Double.NaN;
    this.feeding = false;
  }

  public boolean isSolving() {
    return solving;
  }

  public long getLastSolveDurationMs() {
    return lastSolveDurationMs;
  }

  public void shutdown() {
    running = false;
    synchronized (solveLock) {
      solveLock.notifyAll();
    }
    solverThread.interrupt();
    try {
      solverThread.join(200);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  private void publishInputs(ShotInputs inputs) {
    synchronized (solveLock) {
      pendingInputs = inputs;
      solveLock.notify();
    }
  }

  private void solveLoop() {
    while (running) {
      ShotInputs inputs;
      synchronized (solveLock) {
        while (running && pendingInputs == null) {
          try {
            solveLock.wait();
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
          }
        }
        if (!running) {
          return;
        }
        inputs = pendingInputs;
        pendingInputs = null;
      }

      solving = true;
      long startNanos = System.nanoTime();

      ShotSolution result;
      try {
        result =
            ShotSolver.solve(
                inputs, ShotTable.fromConfig(), BallisticsParameters.fromConfig(), lastFlightTime);
      } catch (RuntimeException e) {
        result =
            new ShotSolution(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, "Shot table error: " + e.getMessage());
      }
      if (result.predictedFlightTimeSec() > 0) {
        lastFlightTime = result.predictedFlightTimeSec();
      }

      lastSolveDurationMs = (System.nanoTime() - startNanos) / 1_000_000;
      lastSolution = result;
      solving = false;
    }
  }

  public CommandBuilder aimAndShootCommand(Follower follower, Sentinel sentinel) {
    return Command.build()
        .setStart(
            () -> {
              follower.holdPoint(follower.getPose());
              startShot(Shooter.constantPower(), true);
            })
        .setDone(() -> !sentinel.isLaunchAllowed(follower.getPose()))
        .setEnd(interrupted -> stopShot())
        .requiring(follower, shooter, turret, intake);
  }

  public CommandBuilder timedAimAndShootCommand(Follower follower, Sentinel sentinel) {
    return lazy(
        () ->
            race(
                aimAndShootCommand(follower, sentinel),
                waitUntil(() -> ballsFired >= config.auto.balls_per_shot_count),
                waitMs(ShotTimeTable.windowMsFor(targetRpmAt(follower.getPose())))));
  }

  public int shotWindowMsAt(Pose pose) {
    return ShotTimeTable.windowMsFor(targetRpmAt(pose));
  }

  private double distanceToGoal(Pose pose) {
    return Math.hypot(
        Field.getGoalY(alliance) - pose.getY(), Field.getGoalX(alliance) - pose.getX());
  }

  private double targetRpmAt(Pose pose) {
    return ShotTable.fromConfig().lookup(distanceToGoal(pose)).rpm();
  }

  public boolean isActive() {
    return active;
  }

  public boolean isFeedCommanded() {
    return feedCommanded;
  }

  private static FlywheelDipDetector newBallDetector() {
    var bd = config.shooter.ball_detection;
    return new FlywheelDipDetector(
        bd.dip_fraction, bd.rebound_fraction, bd.baseline_alpha, bd.refractory_ms);
  }

  public int getBallsFired() {
    return ballsFired;
  }

  public double getElapsedTimeMs() {
    return timer.milliseconds();
  }

  public ShotSolution getLastSolution() {
    return lastSolution;
  }

  public void setInitialSolution(ShotSolution solution) {
    if (solution != null) {
      this.lastSolution = solution;
      shooter.setTargetHoodPosition(solution.targetHoodPosition());
    }
  }

  public double getFlywheelVelocity() {
    return Math.abs(shooter.getShooterVelocity());
  }

  public double getFlywheelTarget() {
    double commandedPower = shooter.getTargetPower();
    return commandedPower > 0
        ? Math.abs(config.shooter.max_rpm * commandedPower)
        : Math.abs(config.shooter.constant_rpm);
  }

  public boolean isFlywheelReady() {
    var shooterConfig = config.shooter;
    double target = getFlywheelTarget();
    double current = getFlywheelVelocity();
    return current >= target * shooterConfig.min_transfer_threshold
        && current <= target * shooterConfig.max_velocity_threshold;
  }

  public double getFlywheelVelocityRatio() {
    double target = getFlywheelTarget();
    return target > 0 ? getFlywheelVelocity() / target : 0.0;
  }

  public String getFlywheelGateDetail() {
    var shooterConfig = config.shooter;
    double ratio = getFlywheelVelocityRatio();
    if (ratio < shooterConfig.min_transfer_threshold) {
      return String.format("BELOW min (%.2fx < %.2f)", ratio, shooterConfig.min_transfer_threshold);
    }
    if (ratio > shooterConfig.max_velocity_threshold) {
      return String.format("ABOVE max (%.2fx > %.2f)", ratio, shooterConfig.max_velocity_threshold);
    }
    return String.format("PASS (%.2fx)", ratio);
  }

  private boolean updateFeedGate() {
    var shooterConfig = config.shooter;
    feeding =
        shouldFeed(
            getFlywheelVelocityRatio(),
            feeding,
            shooterConfig.min_transfer_threshold,
            shooterConfig.max_velocity_threshold,
            shooterConfig.feed_release_threshold);
    return feeding;
  }

  public static boolean shouldFeed(
      double velocityRatio,
      boolean currentlyFeeding,
      double armLow,
      double armHigh,
      double releaseLow) {
    return currentlyFeeding
        ? velocityRatio >= releaseLow
        : velocityRatio >= armLow && velocityRatio <= armHigh;
  }

  private void applySolvedRpm(ShotSolution solution) {
    if (!useSolvedRpm || !solution.isValid() || solution.targetRpm() <= 0) {
      return;
    }
    double rpm =
        Math.clamp(
            solution.targetRpm(),
            config.shooter.ballistics.preferred_shot_rpm,
            config.shooter.ballistics.max_shot_rpm);

    if (!Double.isNaN(commandedSolvedRpm)
        && Math.abs(rpm - commandedSolvedRpm) < config.shooter.ballistics.rpm_update_deadband) {
      return;
    }
    commandedSolvedRpm = rpm;
    shooter.setTargetPower(rpm / config.shooter.max_rpm);
  }

  public double getCommandedSolvedRpm() {
    return commandedSolvedRpm;
  }

  public void periodic() {
    Pose pose = poseSupplier != null ? poseSupplier.get() : null;
    Pose vel = velocitySupplier != null ? velocitySupplier.get() : new Pose(0, 0, 0);

    boolean needSolve =
        active
            || checkAlignment
            || (turret != null
                && turret.isEnabled()
                && turret.getAimMode() == Turret.AimMode.AIM_AT_GOAL);

    if (pose != null && needSolve) {
      double gx = Field.getGoalX(alliance);
      double gy = Field.getGoalY(alliance);
      publishInputs(new ShotInputs(pose, vel, gx, gy));

      ShotSolution solution = lastSolution;

      shooter.setTargetHoodPosition(solution.targetHoodPosition());
      applySolvedRpm(solution);

      turret.setTargetAzimuth(solution.isValid() ? solution.targetAzimuthRad() : Double.NaN);

      if (casablanca != null) {
        boolean enableChassisAim = active && checkAlignment && solution.isValid();
        casablanca.setArmedAimTarget(solution.targetAzimuthRad(), enableChassisAim);
      }
    }

    if (!active) {
      feeding = false;
      feedCommanded = false;
      return;
    }

    var shooterConfig = config.shooter;
    double targetFlywheelVelocity = getFlywheelTarget();

    boolean isFlywheelReady = updateFeedGate();

    if (!ballDetectionArmed && isFlywheelReady) {
      ballDetectionArmed = true;
      ballDetector.reset();
    }
    if (ballDetectionArmed
        && ballDetector.update(getFlywheelVelocity(), (long) timer.milliseconds())) {
      ballsFired++;
    }

    boolean isTurretAligned = true;
    if (checkAlignment && pose != null) {
      isTurretAligned = turret.isAimed(pose);
    }

    boolean isSolutionValid = !requireValidSolution || lastSolution.isValid();

    if (telemetry != null) {
      telemetry.addData("ShotController State", active ? "ARMED" : "IDLE");
      telemetry.addData("Gate 1 (Flywheel)", isFlywheelReady);
      telemetry.addData("Flywheel Arm Window", getFlywheelGateDetail());
      telemetry.addData(
          "Feed Latch",
          feeding
              ? String.format(
                  "FEEDING (releases below %.2fx)", shooterConfig.feed_release_threshold)
              : "closed");
      telemetry.addData("Gate 2 (Alignment)", isTurretAligned);
      telemetry.addData("Gate 3 (Solver Valid)", isSolutionValid);
      telemetry.addData("Balls Fired", ballsFired);
      telemetry.addData("Solution Dist (in)", lastSolution.distanceInches());
      telemetry.addData("Target Hood Pos", lastSolution.targetHoodPosition());
      telemetry.addData("Solved RPM", lastSolution.targetRpm());
      telemetry.addData("Commanded RPM", targetFlywheelVelocity);
      telemetry.addData("Solved Exit Vel (in/s)", lastSolution.targetExitVelocityIps());
      telemetry.addData("Solver Thinking", solving);
      telemetry.addData("Last Solve (ms)", lastSolveDurationMs);
      if (!isSolutionValid) {
        telemetry.addData("Solver Reason", lastSolution.validityReason());
      }
    }

    feedCommanded = isFlywheelReady && isTurretAligned && isSolutionValid;
    if (feedCommanded) {
      intake.run(shooterConfig.feed_intake_power, shooterConfig.feed_transfer_power);
    } else {
      intake.stop();
    }
  }
}
