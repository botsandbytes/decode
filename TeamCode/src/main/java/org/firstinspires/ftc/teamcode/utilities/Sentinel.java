package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;

public class Sentinel {
    // ... (Constants same as before) ...
    public static final double ROBOT_WIDTH = 18.0;
    public static final double ROBOT_WIDTH2 = 17.0;

    // RED GOAL ZONE - Block Blue Alliance from entering
    public static final RectangularZone RED_GOAL_ZONE = new RectangularZone(138.0, 144.0, 69.0, 75.0);
    public static final RectangularZone BLUE_GOAL_ZONE = new RectangularZone(0.0, 6.0, 69.0, 75.0);

    // ... (Launch Zones same as before) ...
    private static final PolygonZone LEFT_BIG_LAUNCH_ZONE = new PolygonZone(
        new Point(144, 144), new Point(0, 144), new Point(72, 72)
    );
    private static final PolygonZone RIGHT_SMALL_LAUNCH_ZONE = new PolygonZone(
        new Point(96, 0), new Point(48, 0), new Point(72, 24)
    );

    // ... (Public Methods wrapper) ...
    public static boolean isLaunchAllowed(Pose currentPose) {
        Point[] robotFootprint = calculateSmallRobotFootprint(currentPose);
        return isIntersectingPolygon(robotFootprint, LEFT_BIG_LAUNCH_ZONE.vertices()) ||
               isIntersectingPolygon(robotFootprint, RIGHT_SMALL_LAUNCH_ZONE.vertices());
    }

    // ... (Other wrappers) ...

    public static boolean doesViolateBlueGoal(Point[] robotFootprint) {
        return isIntersectingRectangle(robotFootprint, BLUE_GOAL_ZONE);
    }

    public static boolean doesViolateRedGoal(Point[] robotFootprint) {
        return isIntersectingRectangle(robotFootprint, RED_GOAL_ZONE);
    }

    // ... (Footprint calc same as before) ...

    /**
     * UPDATED: Checks for overlap by testing if points are inside each other.
     * Handles "Donut" case where zone is fully inside robot.
     */
    private static boolean isIntersectingRectangle(Point[] robot, RectangularZone zone) {
        // 1. Is any part of the robot inside the zone?
        for (Point p : robot) {
            if (p.x() >= zone.minX() && p.x() <= zone.maxX() &&
                p.y() >= zone.minY() && p.y() <= zone.maxY()) return true;
        }

        // 2. Is the zone inside the robot? (The Donut Fix)
        // We check the zone corners against the robot polygon
        Point[] zoneCorners = {
            new Point(zone.minX(), zone.minY()),
            new Point(zone.maxX(), zone.minY()),
            new Point(zone.maxX(), zone.maxY()),
            new Point(zone.minX(), zone.maxY())
        };

        return isIntersectingPolygon(robot, zoneCorners);
    }

    private static boolean isIntersectingPolygon(Point[] shapeA, Point[] shapeB) {
        return isntSeparated(shapeA, shapeB) && isntSeparated(shapeB, shapeA);
    }

    private static boolean isntSeparated(Point[] shapeA, Point[] shapeB) {
        for (int i = 0; i < shapeA.length; i++) {
            Point p1 = shapeA[i];
            Point p2 = shapeA[(i + 1) % shapeA.length];
            Point normal = new Point(-(p2.y() - p1.y()), p2.x() - p1.x());

            Projection projA = projectShapeOnAxis(shapeA, normal);
            Projection projB = projectShapeOnAxis(shapeB, normal);

            if (projA.max() < projB.min() || projB.max() < projA.min()) return false;
        }
        return true;
    }

    private static Projection projectShapeOnAxis(Point[] shape, Point axis) {
        double min = Double.MAX_VALUE;
        double max = -Double.MAX_VALUE;
        for (Point p : shape) {
            double dotProduct = (p.x() * axis.x()) + (p.y() * axis.y());
            min = Math.min(min, dotProduct);
            max = Math.max(max, dotProduct);
        }
        return new Projection(min, max);
    }

    public record Point(double x, double y) {}
    public record RectangularZone(double minX, double maxX, double minY, double maxY) {}
    private record PolygonZone(Point... vertices) {}
    private record Projection(double min, double max) {}

    // Helper to keep the rest of your code working
    public static Point[] calculateRobotFootprint(Pose pose) {
        double heading = pose.getHeading();
        double centerX = pose.getX();
        double centerY = pose.getY();
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

    public static Point[] calculateSmallRobotFootprint(Pose pose) {
        double heading = pose.getHeading();
        double centerX = pose.getX();
        double centerY = pose.getY();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double ROBOT_RADIUS = ROBOT_WIDTH2 / 2;
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
}