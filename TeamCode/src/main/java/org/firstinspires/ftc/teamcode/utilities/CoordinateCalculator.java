package org.firstinspires.ftc.teamcode.utilities;

import java.util.Scanner;

public class CoordinateCalculator {

    // Define the fixed goal coordinates
    private static final double GOAL_X = 129.0;
    private static final double GOAL_Y = 131.0;

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        // Get user input for current coordinates
        System.out.println("Enter current X coordinate:");
        double currentX = scanner.nextDouble();
        System.out.println("Enter current Y coordinate:");
        double currentY = scanner.nextDouble();

        // Calculate distance and angle
        double distance = calculateDistance(currentX, currentY, GOAL_X, GOAL_Y);
        // Angle is in degrees from the positive x-axis (standard mathematical angle)
        double angleDegrees = calculateAngle(currentX, currentY, GOAL_X, GOAL_Y);
        // Bearing (compass direction, North is 0/360, East is 90)
        double bearing = calculateBearing(currentX, currentY, GOAL_X, GOAL_Y);


        // Print the results
        System.out.println("\n--- Results ---");
        System.out.println("Goal Coordinates: (" + GOAL_X + ", " + GOAL_Y + ")");
        System.out.println("Current Coordinates: (" + currentX + ", " + currentY + ")");
        System.out.println("Distance to goal: " + distance);
        System.out.println("Angle (degrees from positive X axis): " + angleDegrees);
        System.out.println("Bearing (degrees clockwise from North): " + bearing);
        System.out.println("Should Turn Left: " + shouldTurnLeft(183, 0));

        scanner.close();
    }

    /**
     * Calculates the Euclidean distance between two points (x1, y1) and (x2, y2).
     * Uses the Pythagorean theorem formula.
     * @return the distance
     */
    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        double deltaX = x2 - x1;
        double deltaY = y2 - y1;
        // Uses Math.sqrt() and Math.pow() or simply multiplication
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static boolean shouldTurnLeft(double current, double target) {
        current = (current % 360 + 360) % 360;
        target = (target % 360 + 360) % 360;
        double leftDiff = (target - current + 360) % 360;
        double rightDiff = (current - target + 360) % 360;
        boolean shortestisleft = leftDiff <= rightDiff;
        boolean straddlesForbidden = (current < 240 && target > 300) || (current > 300 && target < 240);
        boolean pathGoesThroughZone = Math.abs(current - target) < 180;
        if (straddlesForbidden && pathGoesThroughZone) {
            return !shortestisleft;
        }
        return shortestisleft;
    }

    /**
     * Calculates the angle in degrees from the first point to the second point,
     * relative to the positive x-axis (standard mathematical angle).
     * @return the angle in degrees (-180 to 180)
     */
    public static double calculateAngle(double x1, double y1, double x2, double y2) {
        double deltaX = x2 - x1;
        double deltaY = y2 - y1;
        // Math.atan2 returns the angle in radians, in the range (-PI, PI]
        double angleRadians = Math.atan2(deltaY, deltaX);
        // Convert radians to degrees
        return Math.toDegrees(angleRadians);
    }

    /**
     * Calculates the bearing in degrees clockwise from North (0 degrees).
     *
     * @return the bearing in degrees (0 to 360)
     */
    public static double calculateBearing(double x1, double y1, double x2, double y2) {
        double angleDegrees = calculateAngle(x1, y1, x2, y2);
        // The angle from calculateAngle is from the positive x-axis (East).
        // North corresponds to the positive y-axis in standard Cartesian,
        // but typically in bearing calculations North is 0 degrees and bearing increases clockwise.
        // The adjustment below converts standard math angle to compass bearing.
        double bearing = (angleDegrees + 90 + 360) % 360;
        return bearing;
    }
}

