package org.firstinspires.ftc.teamcode.utilities;

import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Sentinel {
    public static double ROBOT_WIDTH = ConfigLoader.getDouble("sentinel.robot_width");
    public static double ROBOT_WIDTH2 = ROBOT_WIDTH - 1.0;

    public static double GOAL_SIZE = ConfigLoader.getDouble("sentinel.goals.size");
    public static double GOAL_MIN_Y = ConfigLoader.getDouble("sentinel.goals.min_y");

    public static RectF RED_GOAL_ZONE = new RectF(
        (float) (144.0 - GOAL_SIZE),
        (float) GOAL_MIN_Y,
        144.0f,
        (float) (GOAL_MIN_Y + GOAL_SIZE)
    );

    public static RectF BLUE_GOAL_ZONE = new RectF(
        0.0f,
        (float) GOAL_MIN_Y,
        (float) GOAL_SIZE,
        (float) (GOAL_MIN_Y + GOAL_SIZE)
    );

    private static PolygonZone LEFT_BIG_LAUNCH_ZONE = new PolygonZone(
        new PointF(144f, 144f), new PointF(0f, 144f), new PointF(72f, 72f)
    );
    private static PolygonZone RIGHT_SMALL_LAUNCH_ZONE = new PolygonZone(
        new PointF(96f, 0f), new PointF(48f, 0f), new PointF(72f, 24f)
    );

    private record PolygonZone(PointF... vertices) {}

    public static boolean isLaunchAllowed(Pose currentPose) {
        Path robot = createPath(calculateSmallRobotFootprint(currentPose));
        return intersects(robot, createPath(LEFT_BIG_LAUNCH_ZONE.vertices)) ||
               intersects(robot, createPath(RIGHT_SMALL_LAUNCH_ZONE.vertices));
    }

    public static boolean violatesActiveGoal(PointF[] footprint) {
        return intersects(createPath(footprint), createPath(getProtectedZone()));
    }

    public static boolean doesViolateBlueGoal(PointF[] robotFootprint) {
        return intersects(createPath(robotFootprint), createPath(BLUE_GOAL_ZONE));
    }

    public static boolean doesViolateRedGoal(PointF[] robotFootprint) {
        return intersects(createPath(robotFootprint), createPath(RED_GOAL_ZONE));
    }

    public static RectF getProtectedZone() {
        return Casablanca.CURRENT_ALLIANCE == Casablanca.Alliance.RED 
            ? BLUE_GOAL_ZONE 
            : RED_GOAL_ZONE;
    }

    public static boolean isRotationSafe(Pose currentPose, double turnInput, double lookaheadRad) {
        double predictedDelta = Math.signum(turnInput) * lookaheadRad;
        Pose futurePose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + predictedDelta);
        
        PointF[] futureFootprint = calculateRobotFootprint(futurePose);
        PointF[] currentFootprint = calculateRobotFootprint(currentPose);
        
        return !violatesActiveGoal(futureFootprint) || violatesActiveGoal(currentFootprint);
    }

    public static RectF getRobotBounds(Pose pose) {
        return getProjectedBounds(calculateRobotFootprint(pose));
    }

    public static PointF[] calculateRobotFootprint(Pose pose) {
        return calculateFootprint(pose, ROBOT_WIDTH);
    }

    public static PointF[] calculateSmallRobotFootprint(Pose pose) {
        return calculateFootprint(pose, ROBOT_WIDTH2);
    }

    private static PointF[] calculateFootprint(Pose pose, double width) {
        double heading = pose.getHeading();
        double centerX = pose.getX();
        double centerY = pose.getY();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double radius = width / 2.0;

        double[] xOffsets = {-radius, radius, radius, -radius};
        double[] yOffsets = {-radius, -radius, radius, radius};
        PointF[] corners = new PointF[4];
        for (int i = 0; i < 4; i++) {
            double rotatedX = (xOffsets[i] * cos) - (yOffsets[i] * sin);
            double rotatedY = (xOffsets[i] * sin) + (yOffsets[i] * cos);
            corners[i] = new PointF((float) (centerX + rotatedX), (float) (centerY + rotatedY));
        }
        return corners;
    }

    private static RectF getProjectedBounds(PointF[] footprint) {
        float minX = Float.MAX_VALUE, maxX = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE, maxY = -Float.MAX_VALUE;
        for (PointF p : footprint) {
            minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
            minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
        }
        return new RectF(minX, minY, maxX, maxY);
    }

    private static Path createPath(PointF[] points) {
        Path path = new Path();
        if (points.length == 0) return path;
        path.moveTo(points[0].x, points[0].y);
        for (int i = 1; i < points.length; i++) {
            path.lineTo(points[i].x, points[i].y);
        }
        path.close();
        return path;
    }

    private static Path createPath(RectF zone) {
        Path path = new Path();
        path.addRect(zone, Path.Direction.CW);
        return path;
    }

    private static boolean intersects(Path pathA, Path pathB) {
        Path intersection = new Path();
        return intersection.op(pathA, pathB, Path.Op.INTERSECT) && !intersection.isEmpty();
    }
}