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

  // ShotSolver.solve() is a full trajectory-optimization search: cheap in the common case, but
  // slow enough on some inputs (target beyond what the calibrated model can loft to goal height)
  // that running it synchronously in periodic() would stall the drivetrain/turret writes for the
  // same tick. It instead runs continuously on a dedicated thread; periodic() only ever publishes
  // the latest inputs (a cheap volatile write) and reads back the most recently completed solution,
  // so a slow solve never blocks a control loop tick. lastFlightTime is the fixed-point warm-start
  // seed for ShotSolver's iteration and is only ever touched by the solver thread.
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

  /**
   * Starts a shot. Aimed shots also solve their own flywheel RPM per distance; fixed-power shots
   * (auto's hand-tuned launch powers, manual TeleOp shots) keep exactly the power passed in. Use
   * {@link #startShot(double, boolean, boolean)} to control that independently.
   */
  public void startShot(double power, boolean checkAlignment) {
    startShot(power, checkAlignment, checkAlignment);
  }

  public void startShot(double power, boolean checkAlignment, boolean useSolvedRpm) {
    startShot(power, checkAlignment, useSolvedRpm, true);
  }

  /**
   * @param power initial flywheel power, used until the first solution lands (and for the whole
   *     shot when {@code useSolvedRpm} is false)
   * @param checkAlignment gate feeding on turret alignment, and drive the turret to AIM_AT_GOAL
   * @param useSolvedRpm re-command the flywheel from the solver's per-distance RPM
   * @param requireValidSolution gate feeding on the solver having a usable answer. Pass false only
   *     when the caller is setting hood and RPM itself and the solution is not steering the shot —
   *     notably the calibration OpMode, which exists to measure distances the shot table does not
   *     cover yet and would otherwise be unable to fire at any of them.
   */
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

  /** True while the solver thread is mid-computation on the latest published inputs. */
  public boolean isSolving() {
    return solving;
  }

  /** Wall-clock cost of the most recently completed solve, for telemetry/diagnostics. */
  public long getLastSolveDurationMs() {
    return lastSolveDurationMs;
  }

  /**
   * Stops the solver thread. Must be called once this ShotController is no longer in use (i.e. from
   * the owning OpMode's teardown) or the thread leaks and keeps solving forever, since a new
   * Robot/ShotController is constructed on every OpMode init.
   */
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

  /** Publishes the latest inputs for the solver thread to pick up; drops any unconsumed input. */
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
        // A malformed shot table must not take the whole control loop down mid-match; report it as
        // an invalid solution so the readiness gate refuses to feed instead.
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

  /**
   * The aim-and-shoot command — one implementation, shared by TeleOp and autonomous.
   *
   * <p>Holds the chassis where it stands, drives the turret to {@code AIM_AT_GOAL}, lets the solver
   * pick the hood angle and a per-distance flywheel RPM, and feeds only once all three readiness
   * gates in {@link #periodic()} agree. It ends itself the moment the robot is no longer inside a
   * launch zone.
   *
   * <p>Autonomous used to run a second, strictly worse shot of its own: fixed {@code constant_rpm}
   * power at every distance, the turret left parked at 0 so the shot was only ever aimed by the
   * path's chassis heading, and a feed gated on flywheel speed alone. Every improvement to aiming,
   * the shot table, and lead compensation reached TeleOp and stopped there. There is one shot in
   * this codebase now.
   */
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

  /**
   * The same shot, ended by whichever comes first: the magazine emptying, or a time budget measured
   * for the flywheel RPM it is fired at. Autonomous has no trigger to release, so something has to
   * decide when a scoring cycle is over and the next path may start.
   *
   * <p>Counting balls (see {@link #getBallsFired()}) is the primary signal — a shot that has put
   * {@code auto.balls_per_shot_count} balls into the air is done regardless of how much of its
   * window is left, which is what actually shortened the scoring cycle once the flywheel overshoot
   * that used to eat the third ball's feed time was fixed. The window from {@link ShotTimeTable} is
   * the fallback for everything counting can't cover: a shot whose gates never open, a jam, a
   * miscount. It also covers aiming and spin-up, not just feeding, so a shot that never arms still
   * hands the chassis back instead of hanging the auto.
   *
   * <p>The pose is read when the command <i>starts</i> rather than when the auto is built: the
   * whole sequence is constructed during init from a robot sitting on the wall, where every score
   * pose is still in the future. That is what {@code lazy} is for here.
   */
  public CommandBuilder timedAimAndShootCommand(Follower follower, Sentinel sentinel) {
    return lazy(
        () ->
            race(
                aimAndShootCommand(follower, sentinel),
                waitUntil(() -> ballsFired >= config.auto.balls_per_shot_count),
                waitMs(ShotTimeTable.windowMsFor(targetRpmAt(follower.getPose())))));
  }

  /** Window this shot would be given if it were fired from {@code pose}, for telemetry. */
  public int shotWindowMsAt(Pose pose) {
    return ShotTimeTable.windowMsFor(targetRpmAt(pose));
  }

  private double distanceToGoal(Pose pose) {
    return Math.hypot(
        Field.getGoalY(alliance) - pose.getY(), Field.getGoalX(alliance) - pose.getX());
  }

  /**
   * The flywheel RPM the current {@link ShotTable} would command from {@code pose} — the key {@link
   * ShotTimeTable} is actually looked up on, since a shot's duration tracks the RPM it was fired
   * at, not the distance a (possibly since-recalibrated) table happened to map to that RPM.
   */
  private double targetRpmAt(Pose pose) {
    return ShotTable.fromConfig().lookup(distanceToGoal(pose)).rpm();
  }

  public boolean isActive() {
    return active;
  }

  /**
   * Whether the last {@link #periodic()} actually commanded the intake to feed.
   *
   * <p>Distinct from {@link #isFlywheelReady()}, which is only the first of three gates. A ball
   * cannot leave the robot unless this was true, which makes it the ground truth for anything
   * counting shots: without it, a slow decay in flywheel speed long after the feed stopped looks
   * exactly like a ball passing through.
   */
  public boolean isFeedCommanded() {
    return feedCommanded;
  }

  private static FlywheelDipDetector newBallDetector() {
    var bd = config.shooter.ball_detection;
    return new FlywheelDipDetector(
        bd.dip_fraction, bd.rebound_fraction, bd.baseline_alpha, bd.refractory_ms);
  }

  /**
   * Balls counted out of the flywheel so far this shot, from the speed each one costs it; see
   * {@link FlywheelDipDetector}. Reset to 0 by every {@link #startShot}.
   */
  public int getBallsFired() {
    return ballsFired;
  }

  public double getElapsedTimeMs() {
    return timer.milliseconds();
  }

  public ShotSolution getLastSolution() {
    return lastSolution;
  }

  /** Primes the initial shot solution and target hood position (e.g. during autonomous init). */
  public void setInitialSolution(ShotSolution solution) {
    if (solution != null) {
      this.lastSolution = solution;
      shooter.setTargetHoodPosition(solution.targetHoodPosition());
    }
  }

  /** Current measured flywheel velocity magnitude (ticks/s from shooter1). */
  public double getFlywheelVelocity() {
    return Math.abs(shooter.getShooterVelocity());
  }

  /**
   * Target flywheel velocity magnitude the readiness gate measures against.
   *
   * <p>Derived from what the flywheel is actually commanded to hold rather than from {@code
   * constant_rpm}, so it stays correct for solved-RPM shots and for auto's hand-tuned launch powers
   * alike. Reading the constant instead meant a shot commanded at any other power was gated against
   * a speed it was never asked to reach.
   */
  public double getFlywheelTarget() {
    double commandedPower = shooter.getTargetPower();
    return commandedPower > 0
        ? Math.abs(config.shooter.max_rpm * commandedPower)
        : Math.abs(config.shooter.constant_rpm);
  }

  /**
   * Returns true iff the flywheel velocity magnitude is inside the arm window — the band it must
   * enter before a feed may *start*. This is the stateless spin-up check; the running feed decision
   * additionally carries hysteresis, see {@link #updateFeedGate()}.
   */
  public boolean isFlywheelReady() {
    var shooterConfig = config.shooter;
    double target = getFlywheelTarget();
    double current = getFlywheelVelocity();
    return current >= target * shooterConfig.min_transfer_threshold
        && current <= target * shooterConfig.max_velocity_threshold;
  }

  /** Velocity as a fraction of the commanded target: 1.00 is on target. */
  public double getFlywheelVelocityRatio() {
    double target = getFlywheelTarget();
    return target > 0 ? getFlywheelVelocity() / target : 0.0;
  }

  /** Which side of the arm window the flywheel is on, for telemetry. */
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

  /**
   * Advances the latching feed gate and returns whether the intake may feed this loop. Call exactly
   * once per loop.
   *
   * <p>Starting a feed requires the full arm window ({@code min_transfer_threshold} to {@code
   * max_velocity_threshold}), but sustaining one only requires staying above the lower {@code
   * feed_release_threshold}. Without that hysteresis the gate chattered: every ball that entered
   * the flywheel dragged it below the arm floor, which cut the intake, which let the wheel recover,
   * which re-opened the gate — the intake ran in visible short bursts instead of continuously.
   *
   * <p>The upper bound deliberately does not apply while already feeding. Balls only ever slow the
   * flywheel down, so an over-speed reading mid-feed is the controller overshooting on its way back
   * up from the previous shot, and interrupting the feed for it would reintroduce the same stutter
   * from the other direction.
   */
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

  /**
   * Pure hysteresis decision behind {@link #updateFeedGate()}, split out so the state machine is
   * unit-testable without hardware.
   *
   * @param velocityRatio measured flywheel speed over commanded target; 1.0 is on target
   * @param currentlyFeeding whether the gate is already open
   * @param armLow lower edge of the window required to *open* the gate
   * @param armHigh upper edge of the window required to *open* the gate
   * @param releaseLow speed the gate falls back to once open; below {@code armLow} by design
   */
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

  /**
   * Re-commands the flywheel from a solved per-distance RPM, but only once the change clears {@code
   * rpm_update_deadband}.
   *
   * <p>The deadband is not just smoothing: {@link Shooter#setTargetPower} discards the accumulated
   * anti-windup integral whenever the setpoint moves, so writing a continuously-drifting setpoint
   * every loop would keep the integrator permanently reset and the flywheel permanently short of
   * its target. Holding the setpoint piecewise-constant lets it converge between real changes.
   */
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

  /** Flywheel setpoint currently commanded by the RPM solver, or NaN when it is not driving it. */
  public double getCommandedSolvedRpm() {
    return commandedSolvedRpm;
  }

  /**
   * Called once per control loop in Robot.update(). Publishes shot inputs to the async solver
   * thread and applies its most recently completed solution; enforces 3 independent readiness
   * gates. Never blocks on the solve itself (see the solveLock fields above), so a slow solve
   * delays how fresh the aim is by at most one solve's wall-clock time, but never stalls this tick.
   */
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

      // Continuous hood position update (single-writer pattern)
      shooter.setTargetHoodPosition(solution.targetHoodPosition());

      // Per-distance flywheel speed (single-writer pattern)
      applySolvedRpm(solution);

      // Pass the lead-compensated azimuth to the Turret only once the solver has actually produced
      // one. NaN makes Turret fall back to its own goal bearing, which is what it should aim at
      // while waiting. Publishing the placeholder solution's 0.0 instead pointed the turret at
      // world bearing 0 for the first loop of every shot — a hard slam to the travel stop in a
      // direction unrelated to the goal, before the real solution arrived and swung it back.
      turret.setTargetAzimuth(solution.isValid() ? solution.targetAzimuthRad() : Double.NaN);

      // Chassis armed aim lock (only if active, checkAlignment is enabled, and solution is valid)
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

    // 1. Flywheel Readiness Gate (latching; arms on the full window, releases on the lower bound)
    boolean isFlywheelReady = updateFeedGate();

    // Ball counting piggybacks on the same readiness signal: only start watching for dips once the
    // wheel has reached its arm window at least once this shot. Spin-up is one long ramp from a
    // stopped or idling wheel up to setpoint, and feeding that climb to the detector — whose
    // reference tracks upward instantly — would either score the ramp itself as noise or, worse,
    // leave a stale reference from before the shot armed. Resetting right at arm time gives the
    // detector a clean baseline at the speed the first real ball actually falls from.
    if (!ballDetectionArmed && isFlywheelReady) {
      ballDetectionArmed = true;
      ballDetector.reset();
    }
    if (ballDetectionArmed
        && ballDetector.update(getFlywheelVelocity(), (long) timer.milliseconds())) {
      ballsFired++;
    }

    // 2. Alignment Readiness Gate
    boolean isTurretAligned = true;
    if (checkAlignment && pose != null) {
      isTurretAligned = turret.isAimed(pose);
    }

    // 3. Solution Validity Gate. Skipped for callers driving hood and RPM themselves, so the
    // calibration OpMode can shoot at distances the table does not cover yet — which is the only
    // way to ever add them to it.
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
