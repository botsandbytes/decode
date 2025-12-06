package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.Sentinel.Point;

@Configurable
public class BorderPatrol {

    public enum Alliance { RED, BLUE }
    public enum Axis { X, Y }

    public static Alliance CURRENT_ALLIANCE = Alliance.RED;

    public static double PREDICTION_HORIZON_SECONDS = 0.35;
    public static double SAFETY_BUFFER_INCHES = 2.0;

    private static final double VELOCITY_SCALAR = 20.0;
    private static final double TURN_SCALAR = 2.5;
    private static final double MIN_VELOCITY = 0.01;

    public static double[] adjustDriveInput(
            Pose2D pose,
            double velX,
            double velY,
            double inputStrafe,
            double inputForward,
            double inputTurn
    ) {
        double heading = pose.getHeading(AngleUnit.RADIANS);

        Velocity fieldRequest = toFieldVelocity(heading, inputStrafe, inputForward);
        Velocity scaledRequest = fieldRequest.scale();

        Velocity actualVelocity = new Velocity(velX, velY);
        Velocity dominantX = dominantComponentX(scaledRequest, actualVelocity);
        Velocity dominantY = dominantComponentY(scaledRequest, actualVelocity);

        Point[] drift = predictFootprint(pose, actualVelocity, 0);
        Point[] xProjection = predictFootprint(pose, dominantX, 0);
        Point[] yProjection = predictFootprint(pose, dominantY, 0);
        Point[] rotationProjection = predictFootprint(pose, new Velocity(0, 0), inputTurn);

        // Check hazards using specific velocities to determine direction
        boolean blockX = checkHazard(drift, actualVelocity.x(), Axis.X) ||
                         checkHazard(xProjection, dominantX.x(), Axis.X);

        boolean blockY = checkHazard(drift, actualVelocity.y(), Axis.Y) ||
                         checkHazard(yProjection, dominantY.y(), Axis.Y);

        boolean blockTurn = rotationHazard(rotationProjection, inputTurn);

        Velocity safeVelocity = limitVelocity(fieldRequest, blockX, blockY);
        double safeTurn = blockTurn ? 0.0 : inputTurn;

        return toRobotCentric(heading, safeVelocity, safeTurn);
    }

    private static Velocity toFieldVelocity(double heading, double strafe, double forward) {
        double strafeAngle = heading - Math.PI / 2.0;

        double x = forward * Math.cos(heading) + strafe * Math.cos(strafeAngle);
        double y = forward * Math.sin(heading) + strafe * Math.sin(strafeAngle);

        return new Velocity(x, y);
    }

    private static Velocity dominantComponentX(Velocity a, Velocity b) {
        return Math.abs(a.x()) > Math.abs(b.x())
                ? new Velocity(a.x(), 0)
                : new Velocity(b.x(), 0);
    }

    private static Velocity dominantComponentY(Velocity a, Velocity b) {
        return Math.abs(a.y()) > Math.abs(b.y())
                ? new Velocity(0, a.y())
                : new Velocity(0, b.y());
    }

    private static Point[] predictFootprint(Pose2D pose, Velocity velocity, double turnSpeed) {
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        double projectedX = bufferedPosition(x, velocity.x());
        double projectedY = bufferedPosition(y, velocity.y());
        double projectedHeading = heading + turnSpeed * TURN_SCALAR * PREDICTION_HORIZON_SECONDS;

        Pose2D future = new Pose2D(
                DistanceUnit.INCH,
                projectedX + velocity.x() * PREDICTION_HORIZON_SECONDS,
                projectedY + velocity.y() * PREDICTION_HORIZON_SECONDS,
                AngleUnit.RADIANS,
                projectedHeading
        );

        return Sentinel.calculateRobotFootprint(future);
    }

    private static double bufferedPosition(double position, double velocityComponent) {
        if (Math.abs(velocityComponent) < MIN_VELOCITY) return position;
        return position + Math.signum(velocityComponent) * SAFETY_BUFFER_INCHES;
    }

    private static boolean checkHazard(Point[] footprint, double velocityComponent, Axis axis) {
        return violatesProtection(footprint, velocityComponent, axis);
    }

    private static boolean violatesProtection(Point[] footprint, double velocity, Axis axis) {
        if (CURRENT_ALLIANCE == Alliance.RED) {
             boolean inZone = Sentinel.doesViolateRedProtection(footprint);
             // Red Zone is Top (High Y). Escaping is moving Down (Negative Y).
             boolean isEscaping = (axis == Axis.Y && velocity < 0);
             return inZone && !isEscaping;
        } else {
             boolean inZone = Sentinel.doesViolateBlueProtection(footprint);
             // Blue Zone is Bottom (Low Y). Escaping is moving Up (Positive Y).
             boolean isEscaping = (axis == Axis.Y && velocity > 0);
             return inZone && !isEscaping;
        }
    }

    private static boolean rotationHazard(Point[] footprint, double turnRequest) {
        if (Math.abs(turnRequest) < MIN_VELOCITY) return false;

        return switch (CURRENT_ALLIANCE) {
            case RED -> Sentinel.doesViolateRedProtection(footprint);
            case BLUE -> Sentinel.doesViolateBlueProtection(footprint);
        };
    }

    private static Velocity limitVelocity(Velocity request, boolean blockX, boolean blockY) {
        double x = blockX ? 0 : request.x();
        double y = blockY ? 0 : request.y();
        return new Velocity(x, y);
    }

    private static double[] toRobotCentric(double heading, Velocity v, double turn) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double strafeAngle = heading - Math.PI / 2.0;

        double robotStrafe = v.x() * Math.cos(strafeAngle) + v.y() * Math.sin(strafeAngle);
        double robotForward = v.x() * cos + v.y() * sin;

        return new double[]{robotStrafe, robotForward, turn};
    }

    private record Velocity(double x, double y) {
        Velocity scale() { return new Velocity(x * BorderPatrol.VELOCITY_SCALAR, y * BorderPatrol.VELOCITY_SCALAR); }
    }
}