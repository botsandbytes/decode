package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Sentinel {

    public static final double ROBOT_WIDTH = 18.0;

    // Pedro Pathing Standard Field Size (0 to 144)
    private static final double FIELD_SIZE_X = 144.0;
    private static final double FIELD_SIZE_Y = 144.0;

    // Protection Zone Constants
    private static final double PROTECTED_ZONE_DEPTH = 5.0;

    // Converted Zones
    // Old: Red was Right Wall (Y=72), extending Down.
    // New: Right Wall (X=144), extending Y (24 to 72).
    public static final RectangularZone RED_PROTECTED_STRIP = new RectangularZone(
        FIELD_SIZE_X - PROTECTED_ZONE_DEPTH, // minX (139)
        FIELD_SIZE_X,                        // maxX (144)
        24.0,                                // minY
        72.0                                 // maxY
    );

    // Old: Blue was Left Wall (Y=-72), extending Down.
    // New: Left Wall (X=0), extending Y (24 to 72).
    public static final RectangularZone BLUE_PROTECTED_STRIP = new RectangularZone(
        0.0,                                 // minX
        PROTECTED_ZONE_DEPTH,                // maxX (5)
        24.0,                                // minY
        72.0                                 // maxY
    );

    // Old: Triangle at Top Edge (-X).
    // New: Triangle at Top Edge (+Y).
    private static final PolygonZone LEFT_BIG_LAUNCH_ZONE = new PolygonZone(
        new Point(144, 144), // Top Right
        new Point(0, 144),   // Top Left
        new Point(72, 72)    // Center
    );

    // Old: Triangle at Bottom Edge (+X).
    // New: Triangle at Bottom Edge (Y=0).
    private static final PolygonZone RIGHT_SMALL_LAUNCH_ZONE = new PolygonZone(
        new Point(96, 0),    // Right of center on bottom wall
        new Point(48, 0),    // Left of center on bottom wall
        new Point(72, 24)    // Peak
    );

    public static boolean isLaunchAllowed(Pose2D currentPose) {
        Point[] robotFootprint = calculateRobotFootprint(currentPose);

        boolean inLeftZone = isIntersectingPolygon(robotFootprint, LEFT_BIG_LAUNCH_ZONE.vertices);
        boolean inRightZone = isIntersectingPolygon(robotFootprint, RIGHT_SMALL_LAUNCH_ZONE.vertices);

        return inLeftZone || inRightZone;
    }

    public static boolean doesViolateRedProtection(Point[] robotFootprint) {
        return isIntersectingRectangle(robotFootprint, RED_PROTECTED_STRIP);
    }

    public static boolean doesViolateBlueProtection(Point[] robotFootprint) {
        return isIntersectingRectangle(robotFootprint, BLUE_PROTECTED_STRIP);
    }

    public static Point[] calculateRobotFootprint(Pose2D pose) {
        double heading = pose.getHeading(AngleUnit.RADIANS);
        double centerX = pose.getX(DistanceUnit.INCH);
        double centerY = pose.getY(DistanceUnit.INCH);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double ROBOT_RADIUS = ROBOT_WIDTH / 2;
        double[] xOffsets = {-ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, -ROBOT_RADIUS};
        double[] yOffsets = {-ROBOT_RADIUS, -ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS};

        Point[] corners = new Point[4];
        for (int i = 0; i < 4; i++) {
            double rotatedX = (xOffsets[i] * cos) - (yOffsets[i] * sin);
            double rotatedY = (xOffsets[i] * sin) + (yOffsets[i] * cos);
            corners[i] = new Point(centerX + rotatedX, centerY + rotatedY);
        }
        return corners;
    }

    private static boolean isIntersectingRectangle(Point[] shape, RectangularZone zone) {
        for (Point p : shape) {
            boolean insideX = p.x >= zone.minX && p.x <= zone.maxX;
            boolean insideY = p.y >= zone.minY && p.y <= zone.maxY;
            if (insideX && insideY) return true;
        }
        return false;
    }

    private static boolean isIntersectingPolygon(Point[] shapeA, Point[] shapeB) {
        return !isSeparated(shapeA, shapeB) && !isSeparated(shapeB, shapeA);
    }

    private static boolean isSeparated(Point[] shapeA, Point[] shapeB) {
        for (int i = 0; i < shapeA.length; i++) {
            Point p1 = shapeA[i];
            Point p2 = shapeA[(i + 1) % shapeA.length];
            Point normal = new Point(-(p2.y - p1.y), p2.x - p1.x);

            Projection projA = projectShapeOnAxis(shapeA, normal);
            Projection projB = projectShapeOnAxis(shapeB, normal);

            if (projA.max < projB.min || projB.max < projA.min) return true;
        }
        return false;
    }

    private static Projection projectShapeOnAxis(Point[] shape, Point axis) {
        double min = Double.MAX_VALUE;
        double max = -Double.MAX_VALUE;
        for (Point p : shape) {
            double dotProduct = (p.x * axis.x) + (p.y * axis.y);
            min = Math.min(min, dotProduct);
            max = Math.max(max, dotProduct);
        }
        return new Projection(min, max);
    }

    public static class Point {
        public final double x, y;
        public Point(double x, double y) { this.x = x; this.y = y; }
    }

    public static class RectangularZone {
        public final double minX, maxX, minY, maxY;
        public RectangularZone(double minX, double maxX, double minY, double maxY) {
            this.minX = minX; this.maxX = maxX; this.minY = minY; this.maxY = maxY;
        }
    }

    private static class PolygonZone {
        public final Point[] vertices;
        public PolygonZone(Point... vertices) { this.vertices = vertices; }
    }

    private static class Projection {
        final double min, max;
        Projection(double min, double max) { this.min = min; this.max = max; }
    }
}