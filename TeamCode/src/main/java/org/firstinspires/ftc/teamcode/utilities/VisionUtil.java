package org.firstinspires.ftc.teamcode.utilities;

import android.util.Size;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionUtil {

  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;
  private boolean tagFound = false;

  public void initAprilTag(HardwareMap hardwareMap, boolean useWebcam) {
    var visionConfig = config.vision;
    double posX = visionConfig.camera_position.x;
    double posY = visionConfig.camera_position.y;
    double posZ = visionConfig.camera_position.z;
    double oriYaw = visionConfig.camera_orientation.yaw;
    double oriPitch = visionConfig.camera_orientation.pitch;
    double oriRoll = visionConfig.camera_orientation.roll;

    Position cameraPosition = new Position(DistanceUnit.CM, posX, posY, posZ, 0);
    YawPitchRollAngles cameraOrientation =
        new YawPitchRollAngles(AngleUnit.DEGREES, oriYaw, oriPitch, oriRoll, 0);

    aprilTag =
        new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();

    aprilTag.setDecimation(3);

    VisionPortal.Builder builder =
        new VisionPortal.Builder()
            .setCameraResolution(new Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

    if (useWebcam) {
      builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      builder.setCamera(BuiltinCameraDirection.BACK);
    }
    builder.addProcessor(aprilTag);
    visionPortal = builder.build();

    stopStreaming();
  }

  public Pose updateAprilTagPose() {
    tagFound = false;
    if (aprilTag == null) {
      return null;
    }
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    if (currentDetections == null) {
      return null;
    }

    Pose newPose = null;
    for (AprilTagDetection detection : currentDetections) {
      if (detection != null && detection.metadata != null && detection.robotPose != null) {
        if (!detection.metadata.name.contains("Obelisk")) {
          tagFound = true;
          double detectedX = detection.robotPose.getPosition().x;
          double detectedY = detection.robotPose.getPosition().y;
          double headingRadians = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
          Pose2D visionPose =
              new Pose2D(
                  DistanceUnit.INCH, detectedX, detectedY, AngleUnit.RADIANS, headingRadians);
          Pose pedroPose =
              PoseConverter.pose2DToPose(visionPose, InvertedFTCCoordinates.INSTANCE)
                  .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
          double fieldX = pedroPose.getX() < 0 ? -pedroPose.getX() + 72 : 72 - pedroPose.getX();
          double fieldY = pedroPose.getY() < 0 ? -pedroPose.getY() + 72 : 72 - pedroPose.getY();
          newPose = new Pose(fieldX, fieldY, headingRadians);
          break;
        }
      }
    }
    return newPose;
  }

  public void resumeStreaming() {
    if (visionPortal != null) {
      try {
        visionPortal.resumeStreaming();
      } catch (Exception ignored) {
      }
    }
  }

  public void stopStreaming() {
    if (visionPortal != null) {
      try {
        visionPortal.stopStreaming();
      } catch (Exception ignored) {
      }
    }
  }

  public boolean isTagFound() {
    return tagFound;
  }
}
