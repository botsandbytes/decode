package org.firstinspires.ftc.teamcode.utilities;

import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.config;

@Configurable
public class Sentinel {
  private final double robotWidth;
  private final double goalSize;
  private final double goalMinY;
  private final RectF redGoalZone;
  private final RectF blueGoalZone;
  private final PointF[] leftBigLaunchZone;
  private final PointF[] rightSmallLaunchZone;
  private final Alliance alliance;

  public Sentinel(Alliance alliance) {
    this.alliance = alliance;
    var s = config.sentinel;
    this.robotWidth = s.robot_width;
    this.goalSize = s.goals.size;
    this.goalMinY = s.goals.min_y;

    this.redGoalZone =
        new RectF(
            (float) (144.0 - goalSize), (float) goalMinY, 144.0f, (float) (goalMinY + goalSize));
    this.blueGoalZone =
        new RectF(0.0f, (float) goalMinY, (float) goalSize, (float) (goalMinY + goalSize));
    this.leftBigLaunchZone =
        new PointF[] {new PointF(144f, 144f), new PointF(0f, 144f), new PointF(72f, 72f)};
    this.rightSmallLaunchZone =
        new PointF[] {new PointF(96f, 0f), new PointF(48f, 0f), new PointF(72f, 24f)};
  }

  public double getRobotWidth() {
    return robotWidth;
  }

  public RectF getRedGoalZone() {
    return redGoalZone;
  }

  public RectF getBlueGoalZone() {
    return blueGoalZone;
  }

  public PointF[] getLeftBigLaunchZone() {
    return leftBigLaunchZone;
  }

  public PointF[] getRightSmallLaunchZone() {
    return rightSmallLaunchZone;
  }

  public Alliance getAlliance() {
    return alliance;
  }

  public boolean isLaunchAllowed(Pose currentPose) {
    PointF[] robot = calculateSmallRobotFootprint(currentPose);
    return intersects(robot, leftBigLaunchZone) || intersects(robot, rightSmallLaunchZone);
  }

  public boolean violatesActiveGoal(PointF[] footprint) {
    return intersects(footprint, getRectVertices(getProtectedZone()));
  }

  public boolean doesViolateBlueGoal(PointF[] robotFootprint) {
    return intersects(robotFootprint, getRectVertices(blueGoalZone));
  }

  public boolean doesViolateRedGoal(PointF[] robotFootprint) {
    return intersects(robotFootprint, getRectVertices(redGoalZone));
  }

  public RectF getProtectedZone() {
    return alliance == Alliance.RED ? blueGoalZone : redGoalZone;
  }

  public boolean isRotationSafe(Pose currentPose, double turnInput, double lookaheadRad) {
    double predictedDelta = Math.signum(turnInput) * lookaheadRad;
    Pose futurePose =
        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + predictedDelta);

    PointF[] futureFootprint = calculateRobotFootprint(futurePose);
    PointF[] currentFootprint = calculateRobotFootprint(currentPose);

    return !violatesActiveGoal(futureFootprint) || violatesActiveGoal(currentFootprint);
  }

  public RectF getRobotBounds(Pose pose) {
    return getProjectedBounds(calculateRobotFootprint(pose));
  }

  public PointF[] calculateRobotFootprint(Pose pose) {
    return calculateFootprint(pose, robotWidth);
  }

  public PointF[] calculateSmallRobotFootprint(Pose pose) {
    return calculateFootprint(pose, robotWidth - 1.0);
  }

  private PointF[] calculateFootprint(Pose pose, double width) {
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

  private RectF getProjectedBounds(PointF[] footprint) {
    float minX = Float.MAX_VALUE, maxX = -Float.MAX_VALUE;
    float minY = Float.MAX_VALUE, maxY = -Float.MAX_VALUE;
    for (PointF p : footprint) {
      minX = Math.min(minX, p.x);
      maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    }
    return new RectF(minX, minY, maxX, maxY);
  }

  private Path createPath(PointF[] points) {
    Path path = new Path();
    if (points.length == 0) return path;
    path.moveTo(points[0].x, points[0].y);
    for (int i = 1; i < points.length; i++) {
      path.lineTo(points[i].x, points[i].y);
    }
    path.close();
    return path;
  }

  private PointF[] getRectVertices(RectF rect) {
    return new PointF[] {
      new PointF(rect.left, rect.top),
      new PointF(rect.right, rect.top),
      new PointF(rect.right, rect.bottom),
      new PointF(rect.left, rect.bottom)
    };
  }

  private boolean intersects(PointF[] polyA, PointF[] polyB) {
    Path pathA = createPath(polyA);
    Path pathB = createPath(polyB);
    Path intersection = new Path();
    try {
      if (intersection.op(pathA, pathB, Path.Op.INTERSECT) && !intersection.isEmpty()) {
        return true;
      }
    } catch (Exception e) {
      // Fall back under JVM/Robolectric test environment
    }
    return satIntersects(polyA, polyB);
  }

  private boolean satIntersects(PointF[] polyA, PointF[] polyB) {
    for (int i = 0; i < polyA.length; i++) {
      PointF p1 = polyA[i];
      PointF p2 = polyA[(i + 1) % polyA.length];
      PointF normal = new PointF(p2.y - p1.y, p1.x - p2.x);
      if (isSeparatingAxis(normal, polyA, polyB)) {
        return false;
      }
    }
    for (int i = 0; i < polyB.length; i++) {
      PointF p1 = polyB[i];
      PointF p2 = polyB[(i + 1) % polyB.length];
      PointF normal = new PointF(p2.y - p1.y, p1.x - p2.x);
      if (isSeparatingAxis(normal, polyA, polyB)) {
        return false;
      }
    }
    return true;
  }

  private boolean isSeparatingAxis(PointF axis, PointF[] polyA, PointF[] polyB) {
    double minA = Double.MAX_VALUE, maxA = -Double.MAX_VALUE;
    for (PointF p : polyA) {
      double projection = p.x * axis.x + p.y * axis.y;
      minA = Math.min(minA, projection);
      maxA = Math.max(maxA, projection);
    }

    double minB = Double.MAX_VALUE, maxB = -Double.MAX_VALUE;
    for (PointF p : polyB) {
      double projection = p.x * axis.x + p.y * axis.y;
      minB = Math.min(minB, projection);
      maxB = Math.max(maxB, projection);
    }

    return maxA < minB || maxB < minA;
  }
}
