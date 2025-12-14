package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.Point;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.RectangularZone;

@Configurable
public class BorderPatrol {

    public enum Alliance { RED, BLUE }

    // --- CONFIGURATION ---
    public static Alliance CURRENT_ALLIANCE = Alliance.RED;

    // Distance (inches) at which the robot starts slowing down automatically
    public static double SLOW_DOWN_DISTANCE = 12.0;

    // The closest the robot is allowed to get to the wall (Safety Margin)
    public static double HARD_STOP_DISTANCE = 2.0;

    // Standard scaling factors
    private static final double VELOCITY_SCALAR = 20.0;

    /**
     * Main entry point to sanitize drive inputs.
     */
    public static double[] adjustDriveInput(
            Pose2D pose,
            double currentVelX, // Field Centric X Velocity (measured)
            double currentVelY, // Field Centric Y Velocity (measured)
            double inputStrafe,
            double inputForward,
            double inputTurn
    ) {
        // 1. Calculate the robot's bounding box based on current rotation
        // This handles the "Rotation makes the robot wider" issue
        Point[] footprint = Sentinel.calculateRobotFootprint(pose);
        Bounds robotBounds = getProjectedBounds(footprint);

        // 2. Convert joystick inputs to Field-Centric requested velocity
        double heading = pose.getHeading(AngleUnit.RADIANS);
        Velocity requested = toFieldVelocity(heading, inputStrafe, inputForward);
        // Scale inputs to physical units (approx inches/sec)
        requested = new Velocity(requested.x * VELOCITY_SCALAR, requested.y * VELOCITY_SCALAR);

        // 3. Apply Wall Limits (The "Virtual Spring")
        // We calculate a "Speed Limit" (0.0 to 1.0) for Left and Right movement
        double limitRight = calculateSpeedLimit(robotBounds, Sentinel.RED_PROTECTED_STRIP, true);
        double limitLeft  = calculateSpeedLimit(robotBounds, Sentinel.BLUE_PROTECTED_STRIP, false);

        // 4. Clamp the requested velocity
        // If requesting Positive X (Right), apply Right Limit
        // If requesting Negative X (Left), apply Left Limit
        double safeX = requested.x;

        if (safeX > 0) {
            safeX *= limitRight;
        } else {
            safeX *= limitLeft;
        }

        // 5. Convert back to Robot-Centric for the drivetrain
        return toRobotCentric(heading, new Velocity(safeX, requested.y), inputTurn);
    }

    /**
     * Calculates a speed multiplier (0.0 to 1.0) based on distance to a zone.
     * @param isRightSideZone: True if the zone is on the positive X side (Red), False if negative (Blue).
     */
    private static double calculateSpeedLimit(Bounds robot, RectangularZone zone, boolean isRightSideZone) {
        // First, check if we are physically lined up with the zone in the Y-axis.
        // If we are "above" or "below" the zone, we don't need to stop.
        boolean overlapsY = (robot.maxY >= zone.minY && robot.minY <= zone.maxY);

        if (!overlapsY) return 1.0; // Safe to drive full speed

        double distanceToWall;

        if (isRightSideZone) {
            // Zone is to our Right (High X). Distance = ZoneMinX - RobotMaxX
            distanceToWall = zone.minX - robot.maxX;
        } else {
            // Zone is to our Left (Low X). Distance = RobotMinX - ZoneMaxX
            distanceToWall = robot.minX - zone.maxX;
        }

        // --- THE FORCE FIELD LOGIC ---

        // 1. If we are inside the buffer (or the wall itself), STOP (0.0 speed allowed towards wall)
        if (distanceToWall <= HARD_STOP_DISTANCE) {
            return 0.0;
        }

        // 2. If we are far away, GO (1.0 speed allowed)
        if (distanceToWall > SLOW_DOWN_DISTANCE) {
            return 1.0;
        }

        // 3. If in between, scale linearly.
        // Distance 12 -> 1.0 (100%)
        // Distance 2 -> 0.0 (0%)
        double effectiveDist = distanceToWall - HARD_STOP_DISTANCE;
        double range = SLOW_DOWN_DISTANCE - HARD_STOP_DISTANCE;

        double scale = effectiveDist / range;

        // Clamp result just in case
        return Math.max(0.0, Math.min(1.0, scale));
    }

    // --- Helpers ---

    private static Bounds getProjectedBounds(Point[] footprint) {
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;

        for (Point p : footprint) {
            minX = Math.min(minX, p.x);
            maxX = Math.max(maxX, p.x);
            minY = Math.min(minY, p.y);
            maxY = Math.max(maxY, p.y);
        }
        return new Bounds(minX, maxX, minY, maxY);
    }

    private static Velocity toFieldVelocity(double heading, double strafe, double forward) {
        double strafeAngle = heading - Math.PI / 2.0;
        double x = forward * Math.cos(heading) + strafe * Math.cos(strafeAngle);
        double y = forward * Math.sin(heading) + strafe * Math.sin(strafeAngle);
        return new Velocity(x, y);
    }

    private static double[] toRobotCentric(double heading, Velocity v, double turn) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double strafeAngle = heading - Math.PI / 2.0;

        // Simple rotation matrix inverse
        double robotForward = v.x * cos + v.y * sin;
        double robotStrafe  = v.x * Math.cos(strafeAngle) + v.y * Math.sin(strafeAngle);

        return new double[]{robotStrafe, robotForward, turn};
    }

    private record Velocity(double x, double y) {}
    private record Bounds(double minX, double maxX, double minY, double maxY) {}
}