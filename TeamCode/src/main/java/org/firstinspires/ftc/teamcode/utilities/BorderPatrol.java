package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.Point;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.RectangularZone;

// TODO: Implement more Sentinel Areas (classifier, parking), prevent shooting from not shoot zone, alliance specific zones, fix inverted y, add velocity based slow done & stop boundaries
@Configurable
public class BorderPatrol {

    public enum Alliance { RED, BLUE }
    public static Alliance CURRENT_ALLIANCE = Alliance.RED;
    public static double SLOW_DOWN_DISTANCE = 24.0;
    public static double HARD_STOP_DISTANCE = 10.0;
    private static final double VELOCITY_SCALAR = 1.0;

    public static double[] adjustDriveInput(Pose2D pose, double curX, double curY, double strafe, double forward, double turn) {
        Point[] footprint = Sentinel.calculateRobotFootprint(pose);
        Bounds robotBounds = getProjectedBounds(footprint);

        double heading = pose.getHeading(AngleUnit.RADIANS);
        Velocity requested = toFieldVelocity(heading, strafe, forward);
        requested = new Velocity(requested.x * VELOCITY_SCALAR, requested.y * VELOCITY_SCALAR);

        // Apply limits for both zones
        Velocity safe = applyZoneLimit(robotBounds, Sentinel.RED_PROTECTED_STRIP, requested);
        safe = applyZoneLimit(robotBounds, Sentinel.BLUE_PROTECTED_STRIP, safe);

        return toRobotCentric(heading, safe, turn);
    }

    private static Velocity applyZoneLimit(Bounds robot, RectangularZone zone, Velocity v) {
        double safeX = v.x;
        double safeY = v.y;

        // --- X AXIS PROTECTION ---
        // Only intervene if the robot is vertically aligned with the zone
        boolean inYLane = (robot.maxY > zone.minY && robot.minY < zone.maxY);
        if (inYLane) {
            if (safeX > 0 && robot.maxX <= zone.minX) { // Moving Right towards zone
                double gap = zone.minX - robot.maxX;
                if (gap < SLOW_DOWN_DISTANCE) safeX *= calculateLimit(gap);
            } else if (safeX < 0 && robot.minX >= zone.maxX) { // Moving Left towards zone
                double gap = robot.minX - zone.maxX;
                if (gap < SLOW_DOWN_DISTANCE) safeX *= calculateLimit(gap);
            } else if (robot.maxX > zone.minX && robot.minX < zone.maxX) {
                // If already partially inside the zone's X-boundary, block movement deeper
                if (safeX > 0 && (robot.minX + robot.maxX)/2 < (zone.minX + zone.maxX)/2) safeX = 0;
                if (safeX < 0 && (robot.minX + robot.maxX)/2 > (zone.minX + zone.maxX)/2) safeX = 0;
            }
        }

        // --- Y AXIS PROTECTION ---
        // Only intervene if the robot is horizontally aligned with the zone
        boolean inXLane = (robot.maxX > zone.minX && robot.minX < zone.maxX);
        if (inXLane) {
            if (safeY > 0 && robot.maxY <= zone.minY) { // Moving Up towards zone
                double gap = zone.minY - robot.maxY;
                if (gap < SLOW_DOWN_DISTANCE) safeY *= calculateLimit(gap);
            } else if (safeY < 0 && robot.minY >= zone.maxY) { // Moving Down towards zone
                double gap = robot.minY - zone.maxY;
                if (gap < SLOW_DOWN_DISTANCE) safeY *= calculateLimit(gap);
            } else if (robot.maxY > zone.minY && robot.minY < zone.maxY) {
                // If already partially inside the zone's Y-boundary, block movement deeper
                if (safeY > 0 && (robot.minY + robot.maxY)/2 < (zone.minY + zone.maxY)/2) safeY = 0;
                if (safeY < 0 && (robot.minY + robot.maxY)/2 > (zone.minY + zone.maxY)/2) safeY = 0;
            }
        }

        return new Velocity(safeX, safeY);
    }

    private static double calculateLimit(double distance) {
        if (distance <= HARD_STOP_DISTANCE) return 0.0;
        if (distance > SLOW_DOWN_DISTANCE) return 1.0;
        return (distance - HARD_STOP_DISTANCE) / (SLOW_DOWN_DISTANCE - HARD_STOP_DISTANCE);
    }

    // ... (Keep existing Helper methods: getProjectedBounds, toFieldVelocity, toRobotCentric) ...
    private static Bounds getProjectedBounds(Point[] footprint) {
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (Point p : footprint) {
            minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
            minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
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
        double robotForward = v.x * cos + v.y * sin;
        double robotStrafe  = v.x * Math.cos(strafeAngle) + v.y * Math.sin(strafeAngle);
        return new double[]{robotStrafe, robotForward, turn};
    }

    private record Velocity(double x, double y) {}
    private record Bounds(double minX, double maxX, double minY, double maxY) {}
}