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

  /**
   * Slack for comparing a distance against a margin. The planner's regions are closed, so their
   * boundary points clear the zone by exactly the margin, and a strict comparison would call the
   * spot it just chose non-compliant. Well below the localizer's resolution.
   */
  private static final double EPSILON = 0.01;

  /**
   * Quadrant segments per buffer arc. JTS approximates the rounded corners of a buffer with an
   * inscribed polygon, which errs on the tight side — the default 8 leaves a planned spot up to
   * ~0.07 in closer to the zone than asked. At 64 that error is a thousandth of an inch.
   */
  private static final int ARC_SEGMENTS = 64;

  /** Where a footprint sits relative to the launch zones, with a margin. */
  public enum ZoneStanding {
    /** Entirely inside a launch zone, margin to spare. */
    INSIDE,
    /** Entirely outside every launch zone, margin to spare. */
    OUTSIDE,
    /** Straddling a boundary, or within the margin of one. The state to get out of. */
    ON_BOUNDARY
  }

  /**
   * Where the robot is standing right now: committed to a launch zone, committed to being out of
   * one, or straddling a boundary.
   *
   * <p>Unlike {@link #nearestEndgameSpot} this reads the actual rotated footprint, so it is the
   * honest answer for the heading the robot is really at — the planner is deliberately blind to
   * heading, this is not.
   */
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

  /**
   * The closest spot where the robot is unambiguously in or unambiguously out of the launch zones —
   * never resting on a boundary — without any part of it crossing the midline into the opponent's
   * half.
   *
   * <p>This is the endgame parking problem. A robot straddling a launch zone line at the buzzer is
   * the worst of both worlds: it is not legally in the zone and it is standing in the way of it.
   * Both committed states are acceptable, so both are offered to the search and the nearer one wins
   * — which is why the returned spot is sometimes <i>deeper into</i> a zone rather than out of it,
   * and why the caller is told which it got: a robot that ends up parked inside a launch zone may
   * as well spend the remaining seconds shooting from it.
   *
   * <p>Only the big triangle can host the "inside" option. Committing to the small far triangle
   * would need the whole footprint plus margin to fit inside it, and it is not big enough for a
   * robot this size — the erosion below simply deletes it, with no special case needed. See {@code
   * MathSafetyTest#testSmallLaunchZoneCannotContainTheRobot}.
   *
   * <p>Clearance is measured against the footprint's half-<i>diagonal</i>, not half-width, so the
   * answer is heading-independent: the robot may be rotated any way at all when the endgame timer
   * fires, and there is no time left to turn it to a friendlier angle.
   *
   * @param currentPose where the robot is now
   * @param margin inches of commitment required on whichever side of the boundary it picks
   * @return the parking spot (holding the current heading) and whether it is inside a launch zone,
   *     or {@code null} if neither state is reachable within this alliance's half
   */
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

  /**
   * The rectangle of robot centers that keep the whole footprint inside this alliance's half of the
   * field, inset by {@code footprintRadius} from the walls and from the midline.
   */
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
