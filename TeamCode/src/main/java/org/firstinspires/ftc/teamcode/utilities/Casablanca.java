package org.firstinspires.ftc.teamcode.utilities;

import android.graphics.RectF;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PredictiveBrakingController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.config.config;

@Configurable
public class Casablanca {

  public boolean enableFrictionComp = true;
  public double frictionX;
  public double frictionY;
  public double frictionRot;

  public boolean enableInputSmoothing = true;
  public double smoothTime;
  public boolean extraSmoothBackLift = true;
  public double backLiftMultiplier;

  public double wallRepulsionPower;
  private double decelSafetyFactor;

  public boolean enableDepthProtection = true;
  public double depthSlowDown;
  public double depthHardStop;

  public boolean enableSideProtection = true;
  public double sideSlowDown;
  public double sideHardStop;

  public double laneBlendDistance;
  private double rotationLookaheadRad;

  public boolean enableHeadingLock;
  public double headingLockKp;
  public double headingLockMaxPower;
  public double headingLockDeadband;

  private double targetHeading = 0.0;
  private boolean isLockActive = false;

  private final NanoTimer timer = new NanoTimer();
  private double currentForward = 0;
  private double currentStrafe = 0;
  private double currentTurn = 0;

  private final Sentinel sentinel;

  public Casablanca(Sentinel sentinel) {
    this.sentinel = sentinel;

    var c = config.casablanca;
    this.frictionX = c.friction.x;
    this.frictionY = c.friction.y;
    this.frictionRot = c.friction.rot;

    this.smoothTime = c.smoothing.time;
    this.backLiftMultiplier = c.smoothing.back_lift_multiplier;

    this.wallRepulsionPower = c.repulsion.power;
    this.decelSafetyFactor = c.repulsion.decel_safety_factor;

    this.depthSlowDown = c.depth.slow_down;
    this.depthHardStop = c.depth.hard_stop;

    this.sideSlowDown = c.side.slow_down;
    this.sideHardStop = c.side.hard_stop;

    this.laneBlendDistance = c.lane_blend_distance;
    this.rotationLookaheadRad = config.sentinel.rotation_lookahead;

    var hl = c.heading_lock;
    this.enableHeadingLock = hl.enabled;
    this.headingLockKp = hl.kp;
    this.headingLockMaxPower = hl.max_power;
    this.headingLockDeadband = hl.deadband;

    reset();
  }

  public final void reset() {
    timer.resetTimer();
    currentForward = 0;
    currentStrafe = 0;
    currentTurn = 0;
    isLockActive = false;
  }

  public double[] adjustDriveInput(
      Pose pose, Vector currentVelocity, double strafe, double forward, double turn) {
    if (enableFrictionComp) {
      forward = applyFriction(forward, frictionX);
      strafe = applyFriction(strafe, frictionY);
      turn = applyFriction(turn, frictionRot);
    }

    if (enableHeadingLock) {
      if (Math.abs(turn) > headingLockDeadband) {
        targetHeading = pose.getHeading();
        isLockActive = false;
      } else {
        if (!isLockActive) {
          targetHeading = pose.getHeading();
          isLockActive = true;
        }
        double headingError =
            com.pedropathing.math.MathFunctions.getSmallestAngleDifference(
                pose.getHeading(), targetHeading);
        double turnDir =
            com.pedropathing.math.MathFunctions.getTurnDirection(pose.getHeading(), targetHeading);
        double correction = headingError * turnDir * headingLockKp;
        turn = Math.clamp(correction, -headingLockMaxPower, headingLockMaxPower);
      }
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

    RectF robotBounds = sentinel.getRobotBounds(pose);
    RectF protectedZone = sentinel.getProtectedZone();

    double currentVelX = currentVelocity.getXComponent();
    double currentVelY = currentVelocity.getYComponent();

    double laneFadeY =
        calculateLaneFade(
            robotBounds.top,
            robotBounds.bottom,
            protectedZone.top,
            protectedZone.bottom,
            laneBlendDistance);
    double laneFadeX =
        calculateLaneFade(
            robotBounds.left,
            robotBounds.right,
            protectedZone.left,
            protectedZone.right,
            laneBlendDistance);

    if (enableDepthProtection && laneFadeY > 0) {
      AxisState xState =
          calculateAxisState(
              robotBounds.left,
              robotBounds.right,
              protectedZone.left,
              protectedZone.right,
              currentVelX,
              adjFieldX,
              depthSlowDown,
              depthHardStop);

      adjFieldX *= (1.0 + laneFadeY * (xState.scale - 1.0));
      if (laneFadeY >= 0.9 && xState.triggerRepulsion) {
        adjFieldX = xState.repulsionDir * wallRepulsionPower;
      }
    }

    if (enableSideProtection && laneFadeX > 0) {
      AxisState yState =
          calculateAxisState(
              robotBounds.top,
              robotBounds.bottom,
              protectedZone.top,
              protectedZone.bottom,
              currentVelY,
              adjFieldY,
              sideSlowDown,
              sideHardStop);

      adjFieldY *= (1.0 + laneFadeX * (yState.scale - 1.0));
      if (laneFadeX >= 0.9 && yState.triggerRepulsion) {
        adjFieldY = yState.repulsionDir * wallRepulsionPower;
      }
    }

    if (turn != 0 && !sentinel.isRotationSafe(pose, turn, rotationLookaheadRad)) {
      turn = 0;
    }

    Vector adjField = new Vector();
    adjField.setOrthogonalComponents(adjFieldX, adjFieldY);
    Vector adjRobot = adjField.copy();
    adjRobot.rotateVector(-pose.getHeading());

    return new double[] {adjRobot.getYComponent(), adjRobot.getXComponent(), turn};
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
    double distToStop = -1.0;
    boolean movingTowards = false;
    double repulsionDir = 0;

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
