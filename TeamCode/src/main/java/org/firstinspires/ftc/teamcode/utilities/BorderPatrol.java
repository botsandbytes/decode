package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Kinematics;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.Point;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.RectangularZone;

@Configurable
public class BorderPatrol {

    public enum Alliance { RED, BLUE }
    public static Alliance CURRENT_ALLIANCE = Alliance.RED;

    // --- CONFIGURATION ---

    // Safety Factor (0.0 to 1.0). Lower = More Conservative (Stops earlier).
    // 0.7 means we assume we only have 70% of the braking power the config claims.
    private static double DECEL_SAFETY_FACTOR = 0.7;

    // X-AXIS: Deep protection (Prevent hitting back of net)
    // Needs a larger buffer because approaching speed is usually higher here.
    public static boolean ENABLE_DEPTH_PROTECTION = true;
    public static double DEPTH_SLOW_DOWN = 8.0; // Increased to catch full speed earlier
    public static double DEPTH_HARD_STOP = 10.0;

    // Y-AXIS: Side protection (Prevent entering from side)
    // Tighter tolerances to allow working near the gate/wall.
    public static boolean ENABLE_SIDE_PROTECTION = true;
    public static double SIDE_SLOW_DOWN = 4.0; // Starts slowing 1 ft away
    public static double SIDE_HARD_STOP = 3.0;  // Stops 4 inches away

    // Rotation lookahead (radians)
    private static final double ROTATION_LOOKAHEAD_RAD = 0.45;

    /**
     * Adjusts drive input to prevent entering opponent's goal zone.
     */
    public static double[] adjustDriveInput(Pose pose, Vector currentVelocity, double strafe, double forward, double turn) {
        // 1. Convert Driver Inputs (Robot Centric) to Field Centric Vector
        Vector inputRobot = new Vector(forward, strafe);
        Vector inputField = rotateVector(inputRobot, pose.getHeading());

        double forwardZeroPowerAccel = Constants.followerConstants.getForwardZeroPowerAcceleration();
        double lateralZeroPowerAccel = Constants.followerConstants.getLateralZeroPowerAcceleration();

        Point[] footprint = Sentinel.calculateRobotFootprint(pose);
        Bounds robotBounds = getProjectedBounds(footprint);

        double currentVelX = currentVelocity.getXComponent();
        double currentVelY = currentVelocity.getYComponent();

        RectangularZone protectedZone = (CURRENT_ALLIANCE == Alliance.RED)
            ? Sentinel.BLUE_GOAL_ZONE
            : Sentinel.RED_GOAL_ZONE;

        // 2. CHECK ALIGNMENT
        // We use the larger of the two slow-down distances for the "Loose Lane" check
        // to ensures we are tracking velocity even if approaching diagonally.
        double maxBuffer = Math.max(DEPTH_SLOW_DOWN, SIDE_SLOW_DOWN);

        boolean inXLaneLoose = robotBounds.minX < protectedZone.maxX() + maxBuffer &&
                               robotBounds.maxX > protectedZone.minX() - maxBuffer;

        boolean inYLaneLoose = robotBounds.minY < protectedZone.maxY() + maxBuffer &&
                               robotBounds.maxY > protectedZone.minY() - maxBuffer;

        // 3. CALCULATE LIMITS

        // --- X AXIS LIMIT (Depth) ---
        double scaleX = 1.0;
        if (ENABLE_DEPTH_PROTECTION && inYLaneLoose) {
            scaleX = calculateAxisScale(
                robotBounds.minX, robotBounds.maxX,
                protectedZone.minX(), protectedZone.maxX(),
                currentVelX, inputField.getXComponent(),
                pose.getHeading(), 0, forwardZeroPowerAccel, lateralZeroPowerAccel,
                DEPTH_SLOW_DOWN, DEPTH_HARD_STOP
            );
        }

        // --- Y AXIS LIMIT (Side) ---
        double scaleY = 1.0;
        if (ENABLE_SIDE_PROTECTION && inXLaneLoose) {
            scaleY = calculateAxisScale(
                robotBounds.minY, robotBounds.maxY,
                protectedZone.minY(), protectedZone.maxY(),
                currentVelY, inputField.getYComponent(),
                pose.getHeading(), Math.PI / 2.0, forwardZeroPowerAccel, lateralZeroPowerAccel,
                SIDE_SLOW_DOWN, SIDE_HARD_STOP
            );
        }

        double finalScale = Math.min(scaleX, scaleY);

        // 4. ROTATIONAL PROTECTION
        if (turn != 0 && !isRotationSafe(pose, turn, protectedZone)) {
            turn = 0;
        }

        // 5. APPLY SCALE
        return new double[]{strafe * finalScale, forward * finalScale, turn};
    }

    private static double calculateAxisScale(double botMin, double botMax, double zoneMin, double zoneMax,
                                             double currentVel, double inputVel,
                                             double heading, double axisAngle,
                                             double fwdAccel, double latAccel,
                                             double slowDownDist, double hardStopDist) {

        double distToStop = -1.0;
        boolean movingTowards = false;

        // Check relationship to zone
        if (botMax <= zoneMin) {
            // Robot is to the Left/Bottom of zone
            distToStop = zoneMin - botMax;
            if (currentVel > 0 || inputVel > 0) movingTowards = true;
        }
        else if (botMin >= zoneMax) {
            // Robot is to the Right/Top of zone
            distToStop = botMin - zoneMax;
            if (currentVel < 0 || inputVel < 0) movingTowards = true;
        }
        else {
            // EJECTION LOGIC: Inside the zone
            double zoneCenter = (zoneMin + zoneMax) / 2.0;
            double botCenter = (botMin + botMax) / 2.0;

            // Allow driving OUT, block driving IN
            if (botCenter < zoneCenter && (currentVel > 0 || inputVel > 0)) return 0.0;
            if (botCenter > zoneCenter && (currentVel < 0 || inputVel < 0)) return 0.0;

            return 1.0;
        }

        if (!movingTowards) return 1.0;
        if (distToStop < 0) return 1.0;

        // --- LIMIT 1: PROXIMITY ---
        // "I am close, so I shouldn't press the gas too hard"
        double proximityScale = 1.0;
        if (distToStop <= hardStopDist) {
            proximityScale = 0.0;
        } else if (distToStop < slowDownDist) {
            proximityScale = (distToStop - hardStopDist) / (slowDownDist - hardStopDist);
            // Non-linear curve to make it feel smoother?
            // proximityScale = Math.pow(proximityScale, 1.5);
        }

        // --- LIMIT 2: PHYSICS (The "Full Speed" Fix) ---
        // "I am going too fast to stop, I must cut power NOW"
        double physicsScale = 1.0;
        if (Math.abs(currentVel) > 0.2) { // Threshold to ignore noise
            double decel = calculateEffectiveDecel(heading, axisAngle, fwdAccel, latAccel);

            // APPLY SAFETY FACTOR: Underestimate our brakes to be safe
            decel *= DECEL_SAFETY_FACTOR;

            if (decel < 1e-6) decel = 0.1;

            double brakingDistance = Math.max(0, distToStop - hardStopDist);
            double actualStoppingDistance = Kinematics.getStoppingDistance(Math.abs(currentVel), decel);

            if (actualStoppingDistance > brakingDistance) {
                // We cannot stop in time if we keep coasting!
                // Immediate cut to 0.
                physicsScale = 0.0;
            } else {
                // Calculate max safe velocity for this distance
                double maxSafeSpeed = Math.sqrt(2 * decel * brakingDistance);
                if (Math.abs(currentVel) > maxSafeSpeed) {
                    physicsScale = maxSafeSpeed / Math.abs(currentVel);
                }
            }
        }

        return Math.min(proximityScale, physicsScale);
    }

    private static boolean isRotationSafe(Pose currentPose, double turnInput, RectangularZone zone) {
        double predictedDelta = Math.signum(turnInput) * ROTATION_LOOKAHEAD_RAD;
        Pose futurePose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + predictedDelta);

        Point[] futureFootprint = Sentinel.calculateRobotFootprint(futurePose);

        boolean willViolate = (zone == Sentinel.BLUE_GOAL_ZONE)
            ? Sentinel.doesViolateBlueGoal(futureFootprint)
            : Sentinel.doesViolateRedGoal(futureFootprint);

        if (willViolate) {
            Point[] currentFootprint = Sentinel.calculateRobotFootprint(currentPose);
            boolean currentlyViolating = (zone == Sentinel.BLUE_GOAL_ZONE)
                ? Sentinel.doesViolateBlueGoal(currentFootprint)
                : Sentinel.doesViolateRedGoal(currentFootprint);

            if (!currentlyViolating) return false;
        }
        return true;
    }

    private static double calculateEffectiveDecel(double heading, double fieldAngle, double fwdAccel, double latAccel) {
        double relativeAngle = fieldAngle - heading;
        double fwdComponent = Math.abs(Math.cos(relativeAngle)) * fwdAccel;
        double latComponent = Math.abs(Math.sin(relativeAngle)) * latAccel;
        return Math.max(fwdComponent, latComponent);
    }

    private static Vector rotateVector(Vector v, double angleRadians) {
        double x = v.getXComponent();
        double y = v.getYComponent();
        return new Vector(
            x * Math.cos(angleRadians) - y * Math.sin(angleRadians),
            x * Math.sin(angleRadians) + y * Math.cos(angleRadians)
        );
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
}