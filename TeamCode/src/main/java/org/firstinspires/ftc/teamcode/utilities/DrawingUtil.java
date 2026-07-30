package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.field.FieldManager;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

public class DrawingUtil {

  public static void drawRobotOnField(
      FieldManager field,
      double x,
      double y,
      double h,
      double turretH,
      double goalX,
      double goalY) {
    if (field == null) return;

    field.setStyle("none", "cyan", 2.0);
    field.moveCursor(x, y);
    field.line(goalX, goalY);

    double cosA = Math.cos(h);
    double sinA = Math.sin(h);
    double halfSize = config.sentinel.robot_width / 2.0;

    double xFL = x + (halfSize * cosA - halfSize * sinA);
    double yFL = y + (halfSize * sinA + halfSize * cosA);

    double xFR = x + (halfSize * cosA - (-halfSize) * sinA);
    double yFR = y + (halfSize * sinA + (-halfSize) * cosA);

    double xBR = x + (-halfSize * cosA - (-halfSize) * sinA);
    double yBR = y + (-halfSize * sinA + (-halfSize) * cosA);

    double xBL = x + (-halfSize * cosA - halfSize * sinA);
    double yBL = y + (-halfSize * sinA + halfSize * cosA);

    field.setStyle("none", "red", 2.0);
    field.moveCursor(xFL, yFL);
    field.line(xFR, yFR);
    field.moveCursor(xFR, yFR);
    field.line(xBR, yBR);
    field.moveCursor(xBR, yBR);
    field.line(xBL, yBL);
    field.moveCursor(xBL, yBL);
    field.line(xFL, yFL);

    double frontX = x + (halfSize * cosA);
    double frontY = y + (halfSize * sinA);

    field.setStyle("none", "yellow", 2.0);
    field.moveCursor(x, y);
    field.line(frontX, frontY);

    double turretCos = Math.cos(turretH);
    double turretSin = Math.sin(turretH);

    double turretX = x + (12.0 * turretCos);
    double turretY = y + (12.0 * turretSin);

    field.setStyle("none", "green", 2.0);
    field.moveCursor(x, y);
    field.line(turretX, turretY);

    field.update();
  }

  public static void drawCasablancaZones(FieldManager field, Sentinel sentinel) {
    if (field == null || sentinel == null) return;
    field.setStyle("stroke", "red", 1.0);
    drawRect(
        field,
        sentinel.getRedGoalZone().getMinX(),
        sentinel.getRedGoalZone().getMinY(),
        sentinel.getRedGoalZone().getMaxX(),
        sentinel.getRedGoalZone().getMaxY());

    field.setStyle("stroke", "blue", 1.0);
    drawRect(
        field,
        sentinel.getBlueGoalZone().getMinX(),
        sentinel.getBlueGoalZone().getMinY(),
        sentinel.getBlueGoalZone().getMaxX(),
        sentinel.getBlueGoalZone().getMaxY());

    field.setStyle("stroke", "green", 1.0);
    drawPolygon(field, sentinel.getLeftBigLaunchZone());
    drawPolygon(field, sentinel.getRightSmallLaunchZone());
  }

  private static void drawPolygon(
      FieldManager field, org.locationtech.jts.geom.Coordinate[] vertices) {
    if (vertices == null || vertices.length < 2) return;
    for (int i = 0; i < vertices.length; i++) {
      org.locationtech.jts.geom.Coordinate current = vertices[i];
      org.locationtech.jts.geom.Coordinate next = vertices[(i + 1) % vertices.length];
      field.moveCursor(current.x, current.y);
      field.line(next.x, next.y);
    }
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
