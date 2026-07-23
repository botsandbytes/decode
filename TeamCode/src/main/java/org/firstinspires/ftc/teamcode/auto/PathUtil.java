package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * Concise utilities for constructing Pedro Pathing {@link Path} and {@link PathChain} instances
 * directly, bypassing the verbose {@code follower.pathBuilder()} wrapper while preserving exact
 * linear heading interpolation behavior.
 */
public final class PathUtil {

  private PathUtil() {}

  public static PathChain pline(Pose start, Pose end) {
    Path path = new Path(new BezierLine(start, end));
    path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
    return new PathChain(path);
  }

  public static PathChain pline(Pose start, Pose end, double headingEndTime) {
    Path path = new Path(new BezierLine(start, end));
    path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), headingEndTime);
    return new PathChain(path);
  }

  public static PathChain pcurve(Pose start, Pose controlPoint, Pose end) {
    Path path = new Path(new BezierCurve(start, controlPoint, end));
    path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
    return new PathChain(path);
  }

  public static PathChain pcurve(Pose start, Pose controlPoint, Pose end, double headingEndTime) {
    Path path = new Path(new BezierCurve(start, controlPoint, end));
    path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), headingEndTime);
    return new PathChain(path);
  }
}
