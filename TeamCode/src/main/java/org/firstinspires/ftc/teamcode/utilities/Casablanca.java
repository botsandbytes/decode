package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.Point;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.RectangularZone;

@Configurable
public class Casablanca {

    public enum Alliance { RED, BLUE }
    public static Alliance CURRENT_ALLIANCE = Alliance.RED;

    // --- SMOOTHING CONFIGURATION ---
    public static boolean ENABLE_INPUT_SMOOTHING = true;

    // Increased smooth time slightly to help with pod lifting
    public static double SMOOTH_TIME = 0.22;

    // Aggressive anti-wheelie protection
    public static boolean EXTRA_SMOOTH_BACK_LIFT = true;
    public static double BACK_LIFT_MULTIPLIER = 2.5;

    // --- BORDER CONFIGURATION ---
    public static double WALL_REPULSION_POWER = 0.12;
    private static double DECEL_SAFETY_FACTOR = 0.7;

    public static boolean ENABLE_DEPTH_PROTECTION = true;
    public static double DEPTH_SLOW_DOWN = 8.0;
    public static double DEPTH_HARD_STOP = 10.0;

    public static boolean ENABLE_SIDE_PROTECTION = true;
    public static double SIDE_SLOW_DOWN = 4.0;
    public static double SIDE_HARD_STOP = 3.0;

    public static double LANE_BLEND_DISTANCE = 12.0;

    private static final double ROTATION_LOOKAHEAD_RAD = 0.45;

    // --- STATE VARIABLES ---
    private static final NanoTimer timer = new NanoTimer();
    private static double currentForward = 0;
    private static double currentStrafe = 0;
    private static double currentTurn = 0;

    /**
     * Resets the smoothing state. CALL THIS IN TELEOP INIT!
     */
    public static void reset() {
        timer.resetTimer();
        currentForward = 0;
        currentStrafe = 0;
        currentTurn = 0;
    }

    /**
     * Adjusts drive input. Returns [strafe, forward, turn]
     */
    public static double[] adjustDriveInput(Pose pose, Vector currentVelocity, double strafe, double forward, double turn) {

        // ------------------------------------------------------------------
        // 1. INPUT SMOOTHING (Fixed Startup Freeze)
        // ------------------------------------------------------------------
        if (ENABLE_INPUT_SMOOTHING) {
            double dt = timer.getElapsedTimeSeconds();
            timer.resetTimer();

            // FIX: If dt is too large (startup lag), clamp it instead of zeroing it.
            if (dt > 0.2) dt = 0.05;

            double maxChange = (1.0 / SMOOTH_TIME) * dt;

            // --- FORWARD SMOOTHING (Anti-Wheelie) ---
            double forwardDiff = forward - currentForward;
            double allowedForwardChange = maxChange;

            // If decelerating or reversing (Front pitching down, Back lifting up)
            if (EXTRA_SMOOTH_BACK_LIFT && forwardDiff < 0) {
                allowedForwardChange = maxChange / BACK_LIFT_MULTIPLIER;
            }

            currentForward = applySlewLimit(currentForward, forward, allowedForwardChange);

            // --- STRAFE & TURN ---
            currentStrafe = applySlewLimit(currentStrafe, strafe, maxChange);
            currentTurn = applySlewLimit(currentTurn, turn, maxChange * 2.5); // Snappy turning

            forward = currentForward;
            strafe = currentStrafe;
            turn = currentTurn;
        }

        // ------------------------------------------------------------------
        // 2. VECTOR CONSTRUCTION (API COMPLIANT FIX)
        // ------------------------------------------------------------------

        // FIX: Used setOrthogonalComponents instead of non-existent setters
        Vector inputRobot = new Vector();
        inputRobot.setOrthogonalComponents(forward, strafe); // X=Forward, Y=Strafe in Pedro

        Vector inputField = rotateVector(inputRobot, pose.getHeading());

        double adjFieldX = inputField.getXComponent();
        double adjFieldY = inputField.getYComponent();

        // ------------------------------------------------------------------
        // 3. Zone Protection LOGIC
        // ------------------------------------------------------------------

        double forwardZeroPowerAccel = Constants.followerConstants.getForwardZeroPowerAcceleration();
        double lateralZeroPowerAccel = Constants.followerConstants.getLateralZeroPowerAcceleration();

        Point[] footprint = Sentinel.calculateRobotFootprint(pose);
        Bounds robotBounds = getProjectedBounds(footprint);

        double currentVelX = currentVelocity.getXComponent();
        double currentVelY = currentVelocity.getYComponent();

        RectangularZone protectedZone = (CURRENT_ALLIANCE == Alliance.RED)
            ? Sentinel.BLUE_GOAL_ZONE
            : Sentinel.RED_GOAL_ZONE;

        // Lane Blending
        double laneFadeY = calculateLaneFade(
            robotBounds.minY, robotBounds.maxY,
            protectedZone.minY(), protectedZone.maxY(), LANE_BLEND_DISTANCE
        );
        double laneFadeX = calculateLaneFade(
            robotBounds.minX, robotBounds.maxX,
            protectedZone.minX(), protectedZone.maxX(), LANE_BLEND_DISTANCE
        );

        // --- X AXIS (Depth) ---
        if (ENABLE_DEPTH_PROTECTION && laneFadeY > 0) {
            AxisState xState = calculateAxisState(
                robotBounds.minX, robotBounds.maxX,
                protectedZone.minX(), protectedZone.maxX(),
                currentVelX, adjFieldX,
                pose.getHeading(), 0, forwardZeroPowerAccel, lateralZeroPowerAccel,
                DEPTH_SLOW_DOWN, DEPTH_HARD_STOP
            );

            double limitX = 1.0 + laneFadeY * (xState.scale - 1.0);
            adjFieldX *= limitX;

            if (laneFadeY >= 0.9 && xState.triggerRepulsion) {
                adjFieldX = xState.repulsionDir * WALL_REPULSION_POWER;
            }
        }

        // --- Y AXIS (Side) ---
        if (ENABLE_SIDE_PROTECTION && laneFadeX > 0) {
            AxisState yState = calculateAxisState(
                robotBounds.minY, robotBounds.maxY,
                protectedZone.minY(), protectedZone.maxY(),
                currentVelY, adjFieldY,
                pose.getHeading(), Math.PI / 2.0, forwardZeroPowerAccel, lateralZeroPowerAccel,
                SIDE_SLOW_DOWN, SIDE_HARD_STOP
            );

            double limitY = 1.0 + laneFadeX * (yState.scale - 1.0);
            adjFieldY *= limitY;

            if (laneFadeX >= 0.9 && yState.triggerRepulsion) {
                adjFieldY = yState.repulsionDir * WALL_REPULSION_POWER;
            }
        }

        // Rotation Protection
        if (turn != 0 && !isRotationSafe(pose, turn, protectedZone)) {
            turn = 0;
        }

        // Convert Back to Robot Centric
        // FIX: Correct API usage
        Vector adjField = new Vector();
        adjField.setOrthogonalComponents(adjFieldX, adjFieldY);

        Vector adjRobot = rotateVector(adjField, -pose.getHeading());

        // Return [Strafe, Forward, Turn] for TeleOp consumption
        return new double[]{adjRobot.getYComponent(), adjRobot.getXComponent(), turn};
    }

    // --- HELPER METHODS ---

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
                                                double heading, double axisAngle,
                                                double fwdAccel, double latAccel,
                                                double slowDownDist, double hardStopDist) {

        AxisState state = new AxisState();
        double distToStop = -1.0;
        boolean movingTowards = false;
        double repulsionDir = 0;

        if (botMax <= zoneMin) {
            distToStop = zoneMin - botMax;
            repulsionDir = -1.0;
            if (currentVel > 0 || inputVel > 0) movingTowards = true;
        }
        else if (botMin >= zoneMax) {
            distToStop = botMin - zoneMax;
            repulsionDir = 1.0;
            if (currentVel < 0 || inputVel < 0) movingTowards = true;
        }
        else {
            double zoneCenter = (zoneMin + zoneMax) / 2.0;
            double botCenter = (botMin + botMax) / 2.0;
            state.triggerRepulsion = true;
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

        // --- LIMIT 1: PROXIMITY (Only applies when close) ---
        double proximityScale = 1.0;
        if (distToStop < slowDownDist) {
            proximityScale = (distToStop - hardStopDist) / (slowDownDist - hardStopDist);
        }

        // --- LIMIT 2: PHYSICS (Always applies) ---
        // This was previously hidden inside the "if (distToStop < slowDownDist)" block.
        // Moving it out allows it to catch high-speed approaches from far away.
        double physicsScale = 1.0;
        if (Math.abs(currentVel) > 0.2) {
            double decel = calculateEffectiveDecel(heading, axisAngle, fwdAccel, latAccel);
            decel *= DECEL_SAFETY_FACTOR;
            if (decel < 1e-6) decel = 0.1;

            double brakingDistance = Math.max(0, distToStop - hardStopDist);
            double maxSafeSpeed = Math.sqrt(2 * decel * brakingDistance);

            if (Math.abs(currentVel) > maxSafeSpeed) {
                physicsScale = maxSafeSpeed / Math.abs(currentVel);
            }
        }

        // Take the most restrictive limit
        state.scale = Math.min(proximityScale, physicsScale);
        return state;
    }

    private static boolean isRotationSafe(Pose currentPose, double turnInput, RectangularZone zone) {
        double predictedDelta = Math.signum(turnInput) * ROTATION_LOOKAHEAD_RAD;
        Pose futurePose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + predictedDelta);
        Point[] futureFootprint = Sentinel.calculateRobotFootprint(futurePose);

        boolean willViolate = (zone == Sentinel.BLUE_GOAL_ZONE)
            ? Sentinel.doesViolateBlueGoal(futureFootprint)
            : Sentinel.doesViolateRedGoal(futureFootprint);

        return !willViolate || ((zone == Sentinel.BLUE_GOAL_ZONE) ? Sentinel.doesViolateBlueGoal(Sentinel.calculateRobotFootprint(currentPose)) : Sentinel.doesViolateRedGoal(Sentinel.calculateRobotFootprint(currentPose)));
    }

    private static double calculateEffectiveDecel(double heading, double fieldAngle, double fwdAccel, double latAccel) {
        double relativeAngle = fieldAngle - heading;
        double fwdComponent = Math.abs(Math.cos(relativeAngle)) * fwdAccel;
        double latComponent = Math.abs(Math.sin(relativeAngle)) * latAccel;
        return Math.max(fwdComponent, latComponent);
    }

    // FIX: Updated to use setOrthogonalComponents to match Vector API
    private static Vector rotateVector(Vector v, double angleRadians) {
        Vector ret = new Vector();
        double newX = v.getXComponent() * Math.cos(angleRadians) - v.getYComponent() * Math.sin(angleRadians);
        double newY = v.getXComponent() * Math.sin(angleRadians) + v.getYComponent() * Math.cos(angleRadians);
        ret.setOrthogonalComponents(newX, newY);
        return ret;
    }

    private static Bounds getProjectedBounds(Point[] footprint) {
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (Point p : footprint) {
            minX = Math.min(minX, p.x()); maxX = Math.max(maxX, p.x());
            minY = Math.min(minY, p.y()); maxY = Math.max(maxY, p.y());
        }
        return new Bounds(minX, maxX, minY, maxY);
    }

    private record Bounds(double minX, double maxX, double minY, double maxY) {}
    private static class AxisState {
        double scale = 1.0;
        boolean triggerRepulsion = false;
        double repulsionDir = 0.0;
    }
}