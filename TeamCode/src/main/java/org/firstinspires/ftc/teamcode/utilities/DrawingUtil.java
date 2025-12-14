package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.field.FieldManager;

public class DrawingUtil {

    /**
     * Draws the robot, heading, and goal line on the Panels dashboard.
     */
    public static void drawRobotOnField(FieldManager field, double x, double y, double h, double goalX, double goalY) {
        if (field == null) return;

        // Visual adjustment: Standard math 0 is Right, but sometimes dashboards expect Up.
        // If your robot draws 90 deg off, remove/add this offset.

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

        // Front Right
        double xFR = x + (halfSize * cosA - (-halfSize) * sinA);
        double yFR = y + (halfSize * sinA + (-halfSize) * cosA);

        // Back Right
        double xBR = x + (-halfSize * cosA - (-halfSize) * sinA);
        double yBR = y + (-halfSize * sinA + (-halfSize) * cosA);

        // Back Left
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
        field.line(xFL, yFL);

        // --- 4. Draw Heading Line (Yellow) ---
        double frontX = x + (halfSize * cosA);
        double frontY = y + (halfSize * sinA);

        field.setStyle("none", "yellow", 2.0);
        field.moveCursor(x, y);
        field.line(frontX, frontY);

        field.update();
    }

    public static void drawBorderPatrolZones(FieldManager field) {
        if (field == null) return;

        // --- RED PROTECTED ZONE (Right Wall) ---
        // Sentinel: X [139, 144], Y [24, 72]
        field.setStyle("stroke", "red", 1.0);
        drawRect(field, 139, 24, 144, 72);

        // --- BLUE PROTECTED ZONE (Left Wall) ---
        // Sentinel: X [0, 5], Y [24, 72]
        field.setStyle("stroke", "blue", 1.0);
        drawRect(field, 0, 24, 5, 72);

        // --- LAUNCH ZONES (Green) ---
        field.setStyle("stroke", "green", 1.0);

        // Big Launch V (Top Edge): (0, 144) -> (144, 144) -> (72, 72)
        field.moveCursor(0, 144);
        field.line(144, 144);
        field.moveCursor(144, 144);
        field.line(72, 72);
        field.moveCursor(72, 72);
        field.line(0, 144);

        // Small Launch Triangle (Bottom Edge): (96, 0) -> (48, 0) -> (72, 24)
        field.moveCursor(96, 0);
        field.line(48, 0);
        field.moveCursor(48, 0);
        field.line(72, 24);
        field.moveCursor(72, 24);
        field.line(96, 0);
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