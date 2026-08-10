package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.EndgameSpot;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.operation.distance.DistanceOp;

@Configurable
public class Sentinel {
  private final double robotWidth;
  private final Envelope redGoalZone;
  private final Envelope blueGoalZone;
  private final Coordinate[] leftBigLaunchZone;
  private final Coordinate[] rightSmallLaunchZone;
  private final Alliance alliance;

  public Sentinel(Alliance alliance) {
    this.alliance = alliance;
    var s = config.sentinel;
    this.robotWidth = s.robot_width;
    double goalSize = s.goals.size;
    double goalMinY = s.goals.min_y;

    this.redGoalZone = new Envelope(144.0 - goalSize, 144.0, goalMinY, goalMinY + goalSize);
    this.blueGoalZone = new Envelope(0.0, goalSize, goalMinY, goalMinY + goalSize);
    this.leftBigLaunchZone =
        new Coordinate[] {
          new Coordinate(144.0, 144.0), new Coordinate(0.0, 144.0), new Coordinate(72.0, 72.0)
        };
    this.rightSmallLaunchZone =
        new Coordinate[] {
          new Coordinate(96.0, 0.0), new Coordinate(48.0, 0.0), new Coordinate(72.0, 24.0)
        };
  }

  public Envelope getRedGoalZone() {
    return redGoalZone;
  }

  public Envelope getBlueGoalZone() {
    return blueGoalZone;
  }

  public Coordinate[] getLeftBigLaunchZone() {
    return leftBigLaunchZone;
  }

  public Coordinate[] getRightSmallLaunchZone() {
    return rightSmallLaunchZone;
  }

  public Alliance getAlliance() {
    return alliance;
  }

  public boolean isLaunchAllowed(Pose currentPose) {
    Coordinate[] robot = calculateSmallRobotFootprint(currentPose);
    return intersects(robot, leftBigLaunchZone) || intersects(robot, rightSmallLaunchZone);
  }

  private static final double EPSILON = 0.01;
  private static final int ARC_SEGMENTS = 64;

  public enum ZoneStanding {
    INSIDE,
    OUTSIDE,
    ON_BOUNDARY
  }

  public ZoneStanding zoneStanding(Pose pose, double margin) {
    Polygon footprint = createJTSPolygon(calculateRobotFootprint(pose));
    Geometry zones = launchZones();

    if (zones.covers(footprint) && footprint.distance(zones.getBoundary()) >= margin - EPSILON) {
      return ZoneStanding.INSIDE;
    }
    if (!zones.intersects(footprint) && footprint.distance(zones) >= margin - EPSILON) {
      return ZoneStanding.OUTSIDE;
    }
    return ZoneStanding.ON_BOUNDARY;
  }

  public EndgameSpot nearestEndgameSpot(Pose currentPose, double margin) {
    double halfDiagonal = robotWidth * Math.sqrt(2.0) / 2.0;
    double reach = halfDiagonal + margin;

    Geometry zones = launchZones();
    Geometry ownHalf = GEOMETRY_FACTORY.toGeometry(ownHalf(halfDiagonal));

    Geometry fullyOutside = ownHalf.difference(zones.buffer(reach, ARC_SEGMENTS));
    Geometry fullyInside = ownHalf.intersection(zones.buffer(-reach, ARC_SEGMENTS));

    Point here =
        GEOMETRY_FACTORY.createPoint(new Coordinate(currentPose.getX(), currentPose.getY()));

    Coordinate outTarget = nearestCoordinate(fullyOutside, here);
    Coordinate inTarget = nearestCoordinate(fullyInside, here);

    if (outTarget == null && inTarget == null) {
      return null;
    }

    boolean takeInside =
        outTarget == null
            || (inTarget != null
                && here.getCoordinate().distance(inTarget)
                    < here.getCoordinate().distance(outTarget));

    Coordinate target = takeInside ? inTarget : outTarget;
    return new EndgameSpot(new Pose(target.x, target.y, currentPose.getHeading()), takeInside);
  }

  private Coordinate nearestCoordinate(Geometry region, Point from) {
    if (region.isEmpty()) {
      return null;
    }
    return region.covers(from) ? from.getCoordinate() : DistanceOp.nearestPoints(region, from)[0];
  }

  private Geometry launchZones() {
    return createJTSPolygon(leftBigLaunchZone).union(createJTSPolygon(rightSmallLaunchZone));
  }

  private Envelope ownHalf(double footprintRadius) {
    boolean farSide = config.sentinel.goals.red_goal_x > 72.0 == (alliance == Alliance.RED);
    double minX = farSide ? 72.0 + footprintRadius : footprintRadius;
    double maxX = farSide ? 144.0 - footprintRadius : 72.0 - footprintRadius;
    return new Envelope(minX, maxX, footprintRadius, 144.0 - footprintRadius);
  }

  public boolean violatesActiveGoal(Coordinate[] footprint) {
    return intersects(footprint, getRectVertices(getProtectedZone()));
  }

  public Envelope getProtectedZone() {
    return alliance == Alliance.RED ? blueGoalZone : redGoalZone;
  }

  public boolean isRotationSafe(Pose currentPose, double turnInput, double lookaheadRad) {
    double predictedDelta = Math.signum(turnInput) * lookaheadRad;
    Pose futurePose =
        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + predictedDelta);

    Coordinate[] futureFootprint = calculateRobotFootprint(futurePose);
    Coordinate[] currentFootprint = calculateRobotFootprint(currentPose);

    return !violatesActiveGoal(futureFootprint) || violatesActiveGoal(currentFootprint);
  }

  public Envelope getRobotBounds(Pose pose) {
    return getProjectedBounds(calculateRobotFootprint(pose));
  }

  public Coordinate[] calculateRobotFootprint(Pose pose) {
    return calculateFootprint(pose, robotWidth);
  }

  public Coordinate[] calculateSmallRobotFootprint(Pose pose) {
    return calculateFootprint(pose, robotWidth - 1.0);
  }

  private Coordinate[] calculateFootprint(Pose pose, double width) {
    double heading = pose.getHeading();
    double centerX = pose.getX();
    double centerY = pose.getY();
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double radius = width / 2.0;

    double[] xOffsets = {-radius, radius, radius, -radius};
    double[] yOffsets = {-radius, -radius, radius, radius};
    Coordinate[] corners = new Coordinate[4];
    for (int i = 0; i < 4; i++) {
      double rotatedX = (xOffsets[i] * cos) - (yOffsets[i] * sin);
      double rotatedY = (xOffsets[i] * sin) + (yOffsets[i] * cos);
      corners[i] = new Coordinate(centerX + rotatedX, centerY + rotatedY);
    }
    return corners;
  }

  private Envelope getProjectedBounds(Coordinate[] footprint) {
    double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
    double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
    for (Coordinate p : footprint) {
      minX = Math.min(minX, p.x);
      maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    }
    return new Envelope(minX, maxX, minY, maxY);
  }

  private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

  private Polygon createJTSPolygon(Coordinate[] points) {
    if (points == null || points.length == 0) {
      return GEOMETRY_FACTORY.createPolygon();
    }
    Coordinate[] coords = new Coordinate[points.length + 1];
    for (int i = 0; i < points.length; i++) {
      coords[i] = new Coordinate(points[i].x, points[i].y);
    }
    coords[points.length] = new Coordinate(points[0].x, points[0].y);
    return GEOMETRY_FACTORY.createPolygon(coords);
  }

  private Coordinate[] getRectVertices(Envelope rect) {
    return new Coordinate[] {
      new Coordinate(rect.getMinX(), rect.getMinY()),
      new Coordinate(rect.getMaxX(), rect.getMinY()),
      new Coordinate(rect.getMaxX(), rect.getMaxY()),
      new Coordinate(rect.getMinX(), rect.getMaxY())
    };
  }

  private boolean intersects(Coordinate[] polyA, Coordinate[] polyB) {
    Polygon pA = createJTSPolygon(polyA);
    Polygon pB = createJTSPolygon(polyB);
    return pA.intersects(pB);
  }
}
