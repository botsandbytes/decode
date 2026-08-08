package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PredictiveBrakingController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Envelope;

@Configurable
public class Casablanca {

  public static boolean enableFrictionComp = true;
  public static double frictionX;
  public static double frictionY;
  public static double frictionRot;

  public static boolean enableInputSmoothing = true;
  public static double smoothTime;
  public static boolean extraSmoothBackLift = true;
  public static double backLiftMultiplier;

  public static double wallRepulsionPower;
  public static double decelSafetyFactor;

  public static boolean enableDepthProtection = true;
  public static double depthSlowDown;
  public static double depthHardStop;

  public static boolean enableSideProtection = true;
  public static double sideSlowDown;
  public static double sideHardStop;

  public static double laneBlendDistance;
  public static double rotationLookaheadTimeSeconds;

  public static boolean fieldCentric;
  public static double fieldCentricOffsetRad;

  public static boolean enableHeadingLock;
  public static double headingLockIntentThreshold;
  public static double headingLockKsMoving;
  public static double headingLockMovingSpeedThreshold;
  public static double headingLockMaxPower;
  public static double headingLockErrorDeadbandRad;
  public static double headingLockSettleRateRad;

  private double targetHeading = 0.0;
  private boolean headingLockInitialized = false;
  private final PIDFController headingPidf;

  private final NanoTimer timer = new NanoTimer();
  private double currentForward = 0;
  private double currentStrafe = 0;
  private double currentTurn = 0;

  private final Sentinel sentinel;

  private double lastLookaheadRad = 0.0;
  private boolean lastRotationSafe = true;
  private double lastAngularVelocityUsed = 0.0;

  private double lastLaneFadeX = 0.0;
  private double lastLaneFadeY = 0.0;
  private double lastDepthScale = 1.0;
  private double lastSideScale = 1.0;
  private boolean lastDepthRepulsion = false;
  private boolean lastSideRepulsion = false;
  private boolean lastPoseUntrusted = false;
  private Envelope lastRobotBounds;
  private Envelope lastProtectedZone;

  /**
   * True if the last call saw a non-finite pose and refused to drive rather than steer off an
   * untrusted localizer reading. Surfaced in TeleOp telemetry so a dead drivetrain is diagnosable.
   */
  public boolean wasPoseUntrusted() {
    return lastPoseUntrusted;
  }

  public double getLastLaneFadeX() {
    return lastLaneFadeX;
  }

  public double getLastLaneFadeY() {
    return lastLaneFadeY;
  }

  public double getLastDepthScale() {
    return lastDepthScale;
  }

  public double getLastSideScale() {
    return lastSideScale;
  }

  public boolean getLastDepthRepulsion() {
    return lastDepthRepulsion;
  }

  public boolean getLastSideRepulsion() {
    return lastSideRepulsion;
  }

  public Envelope getLastRobotBounds() {
    return lastRobotBounds;
  }

  public Envelope getLastProtectedZone() {
    return lastProtectedZone;
  }

  public double getLastLookaheadRad() {
    return lastLookaheadRad;
  }

  public boolean getLastRotationSafe() {
    return lastRotationSafe;
  }

  public double getLastAngularVelocityUsed() {
    return lastAngularVelocityUsed;
  }

  public Casablanca(Sentinel sentinel) {
    this.sentinel = sentinel;

    var c = config.casablanca;
    frictionX = c.friction.x;
    frictionY = c.friction.y;
    frictionRot = c.friction.rot;

    smoothTime = c.smoothing.time;
    backLiftMultiplier = c.smoothing.back_lift_multiplier;

    wallRepulsionPower = c.repulsion.power;
    decelSafetyFactor = c.repulsion.decel_safety_factor;

    enableDepthProtection = c.enable_depth_protection;
    depthSlowDown = c.depth.slow_down;
    depthHardStop = c.depth.hard_stop;

    enableSideProtection = c.enable_side_protection;
    sideSlowDown = c.side.slow_down;
    sideHardStop = c.side.hard_stop;

    laneBlendDistance = c.lane_blend_distance;
    rotationLookaheadTimeSeconds = config.sentinel.rotation_lookahead_time;

    fieldCentric = config.teleop.field_centric;
    fieldCentricOffsetRad = Math.toRadians(config.teleop.field_centric_offset_deg);

    var hl = c.heading_lock;
    enableHeadingLock = hl.enabled;
    headingLockIntentThreshold = hl.intent_threshold;
    headingLockKsMoving = hl.ks_moving;
    headingLockMovingSpeedThreshold = hl.moving_speed_threshold;
    headingLockMaxPower = hl.max_power;
    headingLockErrorDeadbandRad = Math.toRadians(hl.error_deadband_deg);
    headingLockSettleRateRad = Math.toRadians(hl.settle_rate_dps);

    this.headingPidf = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());

    performBrakingSanityCheck();

    reset();
  }

  private void performBrakingSanityCheck() {
    PredictiveBrakingController controller =
        new PredictiveBrakingController(Constants.followerConstants.predictiveBrakingCoefficients);
    double maxVelX = Constants.driveConstants.xVelocity;
    double maxVelY = Constants.driveConstants.yVelocity;
    double minBrakingX =
        Math.abs(controller.computeBrakingDisplacement(maxVelX, 1.0)) / decelSafetyFactor;
    double minBrakingY =
        Math.abs(controller.computeBrakingDisplacement(maxVelY, 1.0)) / decelSafetyFactor;

    com.qualcomm.robotcore.util.RobotLog.ii(
        "Casablanca",
        "Init check: depthHardStop=%.2f in (physics stopping dist at max %.1f in/s is %.2f in), sideHardStop=%.2f in (physics stopping dist at max %.1f in/s is %.2f in)",
        depthHardStop,
        maxVelX,
        minBrakingX,
        sideHardStop,
        maxVelY,
        minBrakingY);
  }

  public final void reset() {
    timer.resetTimer();
    currentForward = 0;
    currentStrafe = 0;
    currentTurn = 0;
    headingLockInitialized = false;
    armedAimActive = false;
    if (headingPidf != null) {
      headingPidf.reset();
    }
  }

  private double armedAimHeadingTarget = 0.0;
  private boolean armedAimActive = false;

  public void setArmedAimTarget(double targetHeadingRad, boolean armed) {
    this.armedAimHeadingTarget = targetHeadingRad;
    this.armedAimActive = armed;
  }

  public double[] adjustDriveInput(
      Pose pose,
      Vector currentVelocity,
      double currentAngularVelocity,
      double strafe,
      double forward,
      double turn) {
    return adjustDriveInput(
        pose, currentVelocity, currentAngularVelocity, strafe, forward, turn, turn);
  }

  /**
   * @param turn the shaped (e.g. cubic-curved, speed-scaled) turn power to command
   * @param rawTurnIntent the driver's unshaped stick deflection, used only to decide whether they
   *     are steering at all. Shaping curves compress small deflections towards zero, so testing
   *     intent against the shaped value would read light steering as "hands off".
   */
  public double[] adjustDriveInput(
      Pose pose,
      Vector currentVelocity,
      double currentAngularVelocity,
      double strafe,
      double forward,
      double turn,
      double rawTurnIntent) {
    if (!Double.isFinite(currentAngularVelocity)) {
      currentAngularVelocity = 0.0;
    }
    if (!Double.isFinite(currentVelocity.getXComponent())
        || !Double.isFinite(currentVelocity.getYComponent())) {
      currentVelocity = new Vector();
    }
    boolean poseFinite =
        Double.isFinite(pose.getX())
            && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getHeading());

    // A non-finite pose means the localizer has failed, and every stage below would quietly
    // propagate that: Vector.rotateVector(NaN) yields NaN components even for a zero-magnitude
    // vector, the JTS Envelope comparisons all evaluate false so the zone protections silently
    // disengage, and NaN reaches the drivetrain. Steering off an untrusted heading is worse than
    // not moving, so refuse outright. Turn was already gated this way — isRotationSafe() requires
    // a finite pose — this extends the same rule to translation.
    lastPoseUntrusted = !poseFinite;
    if (!poseFinite) {
      // Drop the slew accumulators so recovery ramps up from rest rather than from whatever was
      // commanded before the localizer dropped out. Deliberately not reset(), which would also
      // clear the armed-aim latch that ShotController owns.
      currentForward = 0;
      currentStrafe = 0;
      currentTurn = 0;
      headingLockInitialized = false;
      return new double[] {0.0, 0.0, 0.0};
    }

    // Field-centric mapping happens first, so everything downstream still sees a robot-frame
    // (forward, strafe) pair. That matters: frictionX/frictionY and the back-lift slew asymmetry
    // are mechanical properties of the drivetrain's forward and strafe axes, and applying them to
    // field-frame components would smear the strafe friction constant across both axes as the
    // robot turns.
    //
    // This rotation is the inverse of the robot->field rotation applied further down, offset by
    // fieldCentricOffsetRad, so the net field-frame output is the driver's stick vector rotated by
    // the offset alone — independent of robot heading.
    if (fieldCentric) {
      Vector stick = new Vector();
      stick.setOrthogonalComponents(forward, strafe);
      stick.rotateVector(fieldCentricOffsetRad - pose.getHeading());
      forward = stick.getXComponent();
      strafe = stick.getYComponent();
    }

    if (enableFrictionComp) {
      forward = applyFriction(forward, frictionX);
      strafe = applyFriction(strafe, frictionY);
      turn = applyFriction(turn, frictionRot);
    }

    boolean stickReleased = Math.abs(rawTurnIntent) < headingLockIntentThreshold;

    if (armedAimActive && poseFinite && Double.isFinite(armedAimHeadingTarget)) {
      targetHeading = armedAimHeadingTarget;
      headingLockInitialized = true;
    }

    if ((armedAimActive || (enableHeadingLock && stickReleased)) && poseFinite) {
      if (!headingLockInitialized) {
        // Capture the heading to hold only once the robot has actually stopped turning. Latching
        // the instant the stick is released picks a heading the robot is still coasting away from,
        // so the lock immediately fights its own momentum and rings before settling.
        if (Math.abs(currentAngularVelocity) < headingLockSettleRateRad) {
          targetHeading = pose.getHeading();
          headingLockInitialized = true;
          headingPidf.reset();
        }
        turn = 0.0;
      } else {
        double headingError = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());

        if (Math.abs(headingError) < headingLockErrorDeadbandRad) {
          // Within tolerance: hold silently, don't let sensor noise near the zero crossing chatter
          // the drivetrain via the signum-based feedforward below.
          headingPidf.reset();
          turn = 0.0;
        } else {
          headingPidf.updateFeedForwardInput(Math.signum(headingError));
          headingPidf.updateError(headingError);

          double speedMag = currentVelocity.getMagnitude();
          double speedRatio = Math.clamp(speedMag / headingLockMovingSpeedThreshold, 0.0, 1.0);
          double ks = frictionRot + speedRatio * (headingLockKsMoving - frictionRot);

          double correction = headingPidf.run() + Math.copySign(ks, headingError);
          turn = Math.clamp(correction, -headingLockMaxPower, headingLockMaxPower);
        }
      }
    } else {
      // Driver is actively steering (or lock disabled): give them full, unfought authority.
      // Re-latch onto whatever heading they leave the robot at next time the stick is released.
      headingLockInitialized = false;
    }

    if (enableInputSmoothing) {
      double dt = timer.getElapsedTimeSeconds();
      timer.resetTimer();
      if (dt > 0.2) dt = 0.05;

      double maxChange = (1.0 / smoothTime) * dt;
      double forwardDiff = forward - currentForward;
      double allowedForwardChange =
          (extraSmoothBackLift && forwardDiff < 0) ? maxChange / backLiftMultiplier : maxChange;

      currentForward = applySlewLimit(currentForward, forward, allowedForwardChange);
      currentStrafe = applySlewLimit(currentStrafe, strafe, maxChange);
      currentTurn = applySlewLimit(currentTurn, turn, maxChange * 2.5);

      forward = currentForward;
      strafe = currentStrafe;
      turn = currentTurn;
    }

    Vector inputRobot = new Vector();
    inputRobot.setOrthogonalComponents(forward, strafe);

    Vector inputField = inputRobot.copy();
    inputField.rotateVector(pose.getHeading());
    double adjFieldX = inputField.getXComponent();
    double adjFieldY = inputField.getYComponent();

    Envelope robotBounds = sentinel.getRobotBounds(pose);
    Envelope protectedZone = sentinel.getProtectedZone();

    double currentVelX = currentVelocity.getXComponent();
    double currentVelY = currentVelocity.getYComponent();

    double laneFadeY =
        calculateLaneFade(
            robotBounds.getMinY(),
            robotBounds.getMaxY(),
            protectedZone.getMinY(),
            protectedZone.getMaxY(),
            laneBlendDistance);
    double laneFadeX =
        calculateLaneFade(
            robotBounds.getMinX(),
            robotBounds.getMaxX(),
            protectedZone.getMinX(),
            protectedZone.getMaxX(),
            laneBlendDistance);

    lastRobotBounds = robotBounds;
    lastProtectedZone = protectedZone;
    lastLaneFadeX = laneFadeX;
    lastLaneFadeY = laneFadeY;
    lastDepthScale = 1.0;
    lastSideScale = 1.0;
    lastDepthRepulsion = false;
    lastSideRepulsion = false;

    if (enableDepthProtection && laneFadeY > 0) {
      AxisState xState =
          calculateAxisState(
              robotBounds.getMinX(),
              robotBounds.getMaxX(),
              protectedZone.getMinX(),
              protectedZone.getMaxX(),
              currentVelX,
              adjFieldX,
              depthSlowDown,
              depthHardStop);

      lastDepthScale = xState.scale;
      adjFieldX *= (1.0 + laneFadeY * (xState.scale - 1.0));
      if (laneFadeY >= 0.9 && xState.triggerRepulsion) {
        adjFieldX = xState.repulsionDir * wallRepulsionPower;
        lastDepthRepulsion = true;
      }
    }

    if (enableSideProtection && laneFadeX > 0) {
      AxisState yState =
          calculateAxisState(
              robotBounds.getMinY(),
              robotBounds.getMaxY(),
              protectedZone.getMinY(),
              protectedZone.getMaxY(),
              currentVelY,
              adjFieldY,
              sideSlowDown,
              sideHardStop);

      lastSideScale = yState.scale;
      adjFieldY *= (1.0 + laneFadeX * (yState.scale - 1.0));
      if (laneFadeX >= 0.9 && yState.triggerRepulsion) {
        adjFieldY = yState.repulsionDir * wallRepulsionPower;
        lastSideRepulsion = true;
      }
    }

    double lookaheadRad = Math.abs(currentAngularVelocity) * rotationLookaheadTimeSeconds;
    lastLookaheadRad = lookaheadRad;
    lastAngularVelocityUsed = currentAngularVelocity;
    boolean rotationSafe =
        poseFinite
            && Double.isFinite(turn)
            && Double.isFinite(lookaheadRad)
            && sentinel.isRotationSafe(pose, turn, lookaheadRad);
    lastRotationSafe = rotationSafe;
    if (turn != 0 && !rotationSafe) {
      turn = 0;
    }

    Vector adjField = new Vector();
    adjField.setOrthogonalComponents(adjFieldX, adjFieldY);

    return new double[] {adjField.getYComponent(), adjField.getXComponent(), turn};
  }

  public static double applyFriction(double input, double kS) {
    if (Math.abs(input) < 0.01) return 0;
    return Math.signum(input) * (kS + Math.abs(input) * (1.0 - kS));
  }

  public static double applySlewLimit(double current, double target, double maxChange) {
    double diff = target - current;
    if (Math.abs(diff) <= maxChange) return target;
    return current + Math.signum(diff) * maxChange;
  }

  public static double calculateLaneFade(
      double botMin, double botMax, double zoneMin, double zoneMax, double buffer) {
    double distToStrict = Math.max(zoneMin - botMax, botMin - zoneMax);
    if (distToStrict <= 0) return 1.0;
    if (distToStrict >= buffer) return 0.0;
    return 1.0 - (distToStrict / buffer);
  }

  private AxisState calculateAxisState(
      double botMin,
      double botMax,
      double zoneMin,
      double zoneMax,
      double currentVel,
      double inputVel,
      double slowDownDist,
      double hardStopDist) {
    AxisState state = new AxisState();
    double distToStop;
    boolean movingTowards = false;
    double repulsionDir;

    if (botMax <= zoneMin) {
      distToStop = zoneMin - botMax;
      repulsionDir = -1.0;
      if (currentVel > 0 || inputVel > 0) movingTowards = true;
    } else if (botMin >= zoneMax) {
      distToStop = botMin - zoneMax;
      repulsionDir = 1.0;
      if (currentVel < 0 || inputVel < 0) movingTowards = true;
    } else {
      state.triggerRepulsion = true;
      double zoneCenter = (zoneMin + zoneMax) / 2.0;
      double botCenter = (botMin + botMax) / 2.0;
      state.repulsionDir = (botCenter < zoneCenter) ? -1.0 : 1.0;
      state.scale = 0.0;
      return state;
    }

    state.repulsionDir = repulsionDir;

    if (!movingTowards) {
      state.scale = 1.0;
      return state;
    }

    if (distToStop <= hardStopDist) {
      state.scale = 0.0;
      state.triggerRepulsion = true;
      return state;
    }

    double proximityScale = 1.0;
    if (distToStop < slowDownDist) {
      proximityScale = (distToStop - hardStopDist) / (slowDownDist - hardStopDist);
    }

    double physicsScale = 1.0;
    if (Math.abs(currentVel) > 0.2) {
      double brakingRoom = Math.max(0, distToStop - hardStopDist);
      PredictiveBrakingController controller =
          new PredictiveBrakingController(
              Constants.followerConstants.predictiveBrakingCoefficients);
      double predictedBrakingDist =
          Math.abs(controller.computeBrakingDisplacement(currentVel, Math.signum(currentVel)));

      predictedBrakingDist /= decelSafetyFactor;

      if (predictedBrakingDist > brakingRoom && predictedBrakingDist > 0) {
        physicsScale = Math.sqrt(brakingRoom / predictedBrakingDist);
      }
    }
    state.scale = Math.min(proximityScale, physicsScale);
    return state;
  }

  private static class AxisState {
    double scale = 1.0;
    boolean triggerRepulsion = false;
    double repulsionDir = 0.0;
  }
}
