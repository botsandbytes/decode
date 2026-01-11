package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.field.FieldManager;

public class DrawingUtil {

    /**
     * Draws the robot, heading, turret heading, and goal line on the Panels dashboard.
     * Added turretH parameter (Radians).
     */
    public static void drawRobotOnField(FieldManager field, double x, double y, double h, double turretH, double goalX, double goalY) {
        if (field == null) return;

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
        // This represents the Drivetrain heading
        double frontX = x + (halfSize * cosA);
        double frontY = y + (halfSize * sinA);

        field.setStyle("none", "yellow", 2.0);
        field.moveCursor(x, y);
        field.line(frontX, frontY);

        // --- 5. Draw Turret Heading (Green) ---
        // This represents the Turret IMU heading
        double turretCos = Math.cos(turretH);
        double turretSin = Math.sin(turretH);

        // Draw the line slightly longer (12) so it is distinct from the yellow robot heading
        double turretX = x + (12.0 * turretCos);
        double turretY = y + (12.0 * turretSin);

        field.setStyle("none", "green", 2.0);
        field.moveCursor(x, y);
        field.line(turretX, turretY);

        field.update();
    }

    public static void drawBorderPatrolZones(FieldManager field) {
        if (field == null) return;
        // ... (Rest of the file remains unchanged) ...
        // --- RED PROTECTED ZONE (Right Wall) ---
        field.setStyle("stroke", "red", 1.0);
        drawRect(field, 139, 24, 144, 72);

        // --- BLUE PROTECTED ZONE (Left Wall) ---
        field.setStyle("stroke", "blue", 1.0);
        drawRect(field, 0, 24, 5, 72);

        // --- LAUNCH ZONES (Green) ---
        field.setStyle("stroke", "green", 1.0);

        // Big Launch V (Top Edge)
        field.moveCursor(0, 144);
        field.line(144, 144);
        field.moveCursor(144, 144);
        field.line(72, 72);
        field.moveCursor(72, 72);
        field.line(0, 144);

        // Small Launch Triangle (Bottom Edge)
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