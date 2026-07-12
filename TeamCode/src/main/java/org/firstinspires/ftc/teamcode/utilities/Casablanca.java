package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PredictiveBrakingController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import android.graphics.PointF;
import android.graphics.RectF;

@Configurable
public class Casablanca {

    public enum Alliance { RED, BLUE }
    public static Alliance CURRENT_ALLIANCE = Alliance.RED;

    public static boolean ENABLE_FRICTION_COMP = true;
    public static double FRICTION_X = ConfigLoader.getDouble("casablanca.friction.x");
    public static double FRICTION_Y = ConfigLoader.getDouble("casablanca.friction.y");
    public static double FRICTION_ROT = ConfigLoader.getDouble("casablanca.friction.rot");

    public static boolean ENABLE_INPUT_SMOOTHING = true;
    public static double SMOOTH_TIME = ConfigLoader.getDouble("casablanca.smoothing.time");
    public static boolean EXTRA_SMOOTH_BACK_LIFT = true;
    public static double BACK_LIFT_MULTIPLIER = ConfigLoader.getDouble("casablanca.smoothing.back_lift_multiplier");

    public static double WALL_REPULSION_POWER = ConfigLoader.getDouble("casablanca.repulsion.power");
    private static double DECEL_SAFETY_FACTOR = ConfigLoader.getDouble("casablanca.repulsion.decel_safety_factor");

    public static boolean ENABLE_DEPTH_PROTECTION = true;
    public static double DEPTH_SLOW_DOWN = ConfigLoader.getDouble("casablanca.depth.slow_down");
    public static double DEPTH_HARD_STOP = ConfigLoader.getDouble("casablanca.depth.hard_stop");

    public static boolean ENABLE_SIDE_PROTECTION = true;
    public static double SIDE_SLOW_DOWN = ConfigLoader.getDouble("casablanca.side.slow_down");
    public static double SIDE_HARD_STOP = ConfigLoader.getDouble("casablanca.side.hard_stop");

    public static double LANE_BLEND_DISTANCE = ConfigLoader.getDouble("casablanca.lane_blend_distance");
    private static final double ROTATION_LOOKAHEAD_RAD = ConfigLoader.getDouble("sentinel.rotation_lookahead");

    public static boolean ENABLE_HEADING_LOCK = ConfigLoader.getBoolean("casablanca.heading_lock.enabled");
    public static double HEADING_LOCK_KP = ConfigLoader.getDouble("casablanca.heading_lock.kp");
    public static double HEADING_LOCK_MAX_POWER = ConfigLoader.getDouble("casablanca.heading_lock.max_power");
    public static double HEADING_LOCK_DEADBAND = ConfigLoader.getDouble("casablanca.heading_lock.deadband");

    private static double targetHeading = 0.0;
    private static boolean isLockActive = false;

    private static final NanoTimer timer = new NanoTimer();
    private static double currentForward = 0;
    private static double currentStrafe = 0;
    private static double currentTurn = 0;

    public static void reset() {
        timer.resetTimer();
        currentForward = 0;
        currentStrafe = 0;
        currentTurn = 0;
        isLockActive = false;
    }

    public static double[] adjustDriveInput(Pose pose, Vector currentVelocity, double strafe, double forward, double turn) {
        if (ENABLE_FRICTION_COMP) {
            forward = applyFriction(forward, FRICTION_X);
            strafe = applyFriction(strafe, FRICTION_Y);
            turn = applyFriction(turn, FRICTION_ROT);
        }

        if (ENABLE_HEADING_LOCK) {
            if (Math.abs(turn) > HEADING_LOCK_DEADBAND) {
                targetHeading = pose.getHeading();
                isLockActive = false;
            } else {
                if (!isLockActive) {
                    targetHeading = pose.getHeading();
                    isLockActive = true;
                }
                double headingError = com.pedropathing.math.MathFunctions.getSmallestAngleDifference(pose.getHeading(), targetHeading);
                double turnDir = com.pedropathing.math.MathFunctions.getTurnDirection(pose.getHeading(), targetHeading);
                double correction = headingError * turnDir * HEADING_LOCK_KP;
                turn = Math.clamp(correction, -HEADING_LOCK_MAX_POWER, HEADING_LOCK_MAX_POWER);
            }
        }

        if (ENABLE_INPUT_SMOOTHING) {
            double dt = timer.getElapsedTimeSeconds();
            timer.resetTimer();
            if (dt > 0.2) dt = 0.05;

            double maxChange = (1.0 / SMOOTH_TIME) * dt;
            double forwardDiff = forward - currentForward;
            double allowedForwardChange = (EXTRA_SMOOTH_BACK_LIFT && forwardDiff < 0)
                ? maxChange / BACK_LIFT_MULTIPLIER 
                : maxChange;

            currentForward = applySlewLimit(currentForward, forward, allowedForwardChange);
            currentStrafe = applySlewLimit(currentStrafe, strafe, maxChange);
            currentTurn = applySlewLimit(currentTurn, turn, maxChange * 2.5);

            forward = currentForward;
            strafe = currentStrafe;
            turn = currentTurn;
        }

        Vector inputRobot = new Vector();
        inputRobot.setOrthogonalComponents(forward, strafe);

        Vector inputField = rotateVector(inputRobot, pose.getHeading());
        double adjFieldX = inputField.getXComponent();
        double adjFieldY = inputField.getYComponent();

        RectF robotBounds = Sentinel.getRobotBounds(pose);
        RectF protectedZone = Sentinel.getProtectedZone();

        double currentVelX = currentVelocity.getXComponent();
        double currentVelY = currentVelocity.getYComponent();

        double laneFadeY = calculateLaneFade(robotBounds.top, robotBounds.bottom, protectedZone.top, protectedZone.bottom, LANE_BLEND_DISTANCE);
        double laneFadeX = calculateLaneFade(robotBounds.left, robotBounds.right, protectedZone.left, protectedZone.right, LANE_BLEND_DISTANCE);

        if (ENABLE_DEPTH_PROTECTION && laneFadeY > 0) {
            AxisState xState = calculateAxisState(
                robotBounds.left, robotBounds.right,
                protectedZone.left, protectedZone.right,
                currentVelX, adjFieldX,
                DEPTH_SLOW_DOWN, DEPTH_HARD_STOP
            );

            adjFieldX *= (1.0 + laneFadeY * (xState.scale - 1.0));
            if (laneFadeY >= 0.9 && xState.triggerRepulsion) {
                adjFieldX = xState.repulsionDir * WALL_REPULSION_POWER;
            }
        }

        if (ENABLE_SIDE_PROTECTION && laneFadeX > 0) {
            AxisState yState = calculateAxisState(
                robotBounds.top, robotBounds.bottom,
                protectedZone.top, protectedZone.bottom,
                currentVelY, adjFieldY,
                SIDE_SLOW_DOWN, SIDE_HARD_STOP
            );

            adjFieldY *= (1.0 + laneFadeX * (yState.scale - 1.0));
            if (laneFadeX >= 0.9 && yState.triggerRepulsion) {
                adjFieldY = yState.repulsionDir * WALL_REPULSION_POWER;
            }
        }

        if (turn != 0 && !Sentinel.isRotationSafe(pose, turn, ROTATION_LOOKAHEAD_RAD)) {
            turn = 0;
        }

        Vector adjField = new Vector();
        adjField.setOrthogonalComponents(adjFieldX, adjFieldY);
        Vector adjRobot = rotateVector(adjField, -pose.getHeading());

        return new double[]{adjRobot.getYComponent(), adjRobot.getXComponent(), turn};
    }

    private static double applyFriction(double input, double kS) {
        if (Math.abs(input) < 0.01) return 0;
        return Math.signum(input) * (kS + Math.abs(input) * (1.0 - kS));
    }

    private static double applySlewLimit(double current, double target, double maxChange) {
        double diff = target - current;
        if (Math.abs(diff) <= maxChange) return target;
        return current + Math.signum(diff) * maxChange;
    }

    private static double calculateLaneFade(double botMin, double botMax, double zoneMin, double zoneMax, double buffer) {
        double distToStrict = Math.max(zoneMin - botMax, botMin - zoneMax);
        if (distToStrict <= 0) return 1.0;
        if (distToStrict >= buffer) return 0.0;
        return 1.0 - (distToStrict / buffer);
    }

    private static AxisState calculateAxisState(double botMin, double botMax, double zoneMin, double zoneMax,
                                                double currentVel, double inputVel,
                                                double slowDownDist, double hardStopDist) {
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
            PredictiveBrakingController controller = new PredictiveBrakingController(Constants.followerConstants.predictiveBrakingCoefficients);
            double predictedBrakingDist = Math.abs(controller.computeBrakingDisplacement(currentVel, Math.signum(currentVel)));

            predictedBrakingDist /= DECEL_SAFETY_FACTOR;

            if (predictedBrakingDist > brakingRoom && predictedBrakingDist > 0) {
                physicsScale = Math.sqrt(brakingRoom / predictedBrakingDist);
            }
        }
        state.scale = Math.min(proximityScale, physicsScale);
        return state;
    }

    private static Vector rotateVector(Vector v, double angleRadians) {
        Vector ret = new Vector();
        double newX = v.getXComponent() * Math.cos(angleRadians) - v.getYComponent() * Math.sin(angleRadians);
        double newY = v.getXComponent() * Math.sin(angleRadians) + v.getYComponent() * Math.cos(angleRadians);
        ret.setOrthogonalComponents(newX, newY);
        return ret;
    }

    private static class AxisState {
        double scale = 1.0;
        boolean triggerRepulsion = false;
        double repulsionDir = 0.0;
    }
}