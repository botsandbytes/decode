package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;

public final class OpModeUtil {

  private OpModeUtil() {}

  public static FieldManager initPanelsField() {
    FieldManager field = PanelsField.INSTANCE.getField();
    field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    return field;
  }

  public static String getBlackboardPoseKey(Alliance alliance) {
    return alliance == Alliance.RED ? "RED_POSE" : "BLUE_POSE";
  }

  public static Pose getSavedPose(Alliance alliance, Pose defaultPose) {
    String key = getBlackboardPoseKey(alliance);
    Pose savedPose = (Pose) OpMode.blackboard.get(key);
    return savedPose != null ? savedPose : defaultPose;
  }

  public static void savePose(Alliance alliance, Pose pose) {
    OpMode.blackboard.put(getBlackboardPoseKey(alliance), pose);
  }

  public static void setupTurretAndShooter(
      Turret turret, Shooter shooter, double initialHeadingRad) {
    turret.setInitialHeading(initialHeadingRad);
    shooter.setShooterPIDFCoefficients();
    turret.setHoldAngle(Math.toDegrees(initialHeadingRad));
    turret.setAimMode(Turret.AimMode.IDLE);
  }

  public static void drawRobot(
      FieldManager field, Follower follower, Turret turret, double goalX, double goalY) {
    DrawingUtil.drawRobotOnField(
        field,
        follower.getPose().getX(),
        follower.getPose().getY(),
        follower.getPose().getHeading(),
        Math.toRadians(turret.getCurrentTurnAngle()),
        goalX,
        goalY);
  }
}
