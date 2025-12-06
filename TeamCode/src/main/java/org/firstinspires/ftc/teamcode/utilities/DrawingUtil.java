package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.field.FieldManager;

public class DrawingUtil {

    /**
     * Draws the robot, heading, and goal line on the Panels dashboard.
     * Explicitly moves cursor between line segments to ensure a square is drawn.
     */
    public static void drawRobotOnField(FieldManager field, double x, double y, double h, double goalX, double goalY) {
        if (field == null) return;

        h += Math.PI / 2;
        // --- 1. Draw Line to Goal (Cyan) ---
        field.setStyle("none", "cyan", 2.0);
        field.moveCursor(x, y);
        field.line(goalX, goalY);

        // --- 2. Calculate Corners (18x18 inches) ---
        double cosA = Math.cos(h);
        double sinA = Math.sin(h);
        double halfSize = 9.0;

        // Front Left
        double xFL = x + (halfSize * cosA - halfSize * sinA);
        double yFL = y + (halfSize * sinA + halfSize * cosA);

        // Front Right (y is negative)
        double xFR = x + (halfSize * cosA - (-halfSize) * sinA);
        double yFR = y + (halfSize * sinA + (-halfSize) * cosA);

        // Back Right (x and y negative)
        double xBR = x + (-halfSize * cosA - (-halfSize) * sinA);
        double yBR = y + (-halfSize * sinA + (-halfSize) * cosA);

        // Back Left (x negative)
        double xBL = x + (-halfSize * cosA - halfSize * sinA);
        double yBL = y + (-halfSize * sinA + halfSize * cosA);

        // --- 3. Draw Robot Body (Red Box) ---
        field.setStyle("none", "red", 2.0);

        field.moveCursor(xFL, yFL);
        field.line(xFR, yFR);

        field.moveCursor(xFR, yFR);
        field.line(xBR, yBR);

        field.moveCursor(xBR, yBR);
        field.line(xBL, yBL);

        field.moveCursor(xBL, yBL);
        field.line(xFL, yFL); // Close the box

        // --- 4. Draw Heading Line (Yellow) ---
        double frontX = x + (halfSize * cosA);
        double frontY = y + (halfSize * sinA);

        field.setStyle("none", "yellow", 2.0);
        field.moveCursor(x, y);
        field.line(frontX, frontY);

        // --- 5. Push Update ---
        field.update();
    }

    public static void drawBorderPatrolZones(FieldManager field) {
        if (field == null) return;

        // --- RED PROTECTED ZONE (Top Edge Strip) ---
        // Sentinel: X [0, 48], Y [67, 72]
        field.setStyle("stroke", "red", 1.0);
        drawRect(field, 0, 67, 48, 72);

        // --- BLUE PROTECTED ZONE (Bottom Edge Strip) ---
        // Sentinel: X [0, 48], Y [-72, -67]
        field.setStyle("stroke", "blue", 1.0);
        drawRect(field, 0, -72, 48, -67);

        // --- LAUNCH ZONES (Yellow) ---
        field.setStyle("stroke", "green", 1.0);

        // Big Launch V (Left Side): (-72, 72) -> (-72, -72) -> (0, 0)
        field.moveCursor(-72, 72);
        field.line(0, 0);
        field.moveCursor(-72, -72);
        field.line(0, 0);
        field.moveCursor(-72, 72); // Close the back
        field.line(-72, -72);

        // Small Launch Triangle (Right Side): (72, 24) -> (72, -24) -> (48, 0)
        field.moveCursor(72, 24);
        field.line(48, 0);
        field.moveCursor(72, -24);
        field.line(48, 0);
        field.moveCursor(72, 24); // Close the back
        field.line(72, -24);
    }

    private static void drawRect(FieldManager field, double x1, double y1, double x2, double y2) {
        field.moveCursor(x1, y1);
        field.line(x2, y1);
        field.moveCursor(x2, y1);
        field.line(x2, y2);
        field.moveCursor(x2, y2);
        field.line(x1, y2);
        field.moveCursor(x1, y2);
        field.line(x1, y1);
    }
}