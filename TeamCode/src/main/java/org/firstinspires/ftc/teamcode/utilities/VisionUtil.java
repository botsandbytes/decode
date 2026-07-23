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
import org.firstinspires.ftc.teamcode.robot.config.config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionUtil {

  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;
  private boolean tagFound = false;

  public void initAprilTag(HardwareMap hardwareMap, boolean useWebcam) {
    var v = config.vision;
    double posX = v.camera_position.x;
    double posY = v.camera_position.y;
    double posZ = v.camera_position.z;
    double oriYaw = v.camera_orientation.yaw;
    double oriPitch = v.camera_orientation.pitch;
    double oriRoll = v.camera_orientation.roll;

    Position cameraPosition = new Position(DistanceUnit.CM, posX, posY, posZ, 0);
    YawPitchRollAngles cameraOrientation =
        new YawPitchRollAngles(AngleUnit.DEGREES, oriYaw, oriPitch, oriRoll, 0);

    aprilTag =
        new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();

    aprilTag.setDecimation(3);

    VisionPortal.Builder builder =
        new VisionPortal.Builder()
            .setCameraResolution(new Size(1920, 1080))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

    if (useWebcam) {
      builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      builder.setCamera(BuiltinCameraDirection.BACK);
    }
    builder.addProcessor(aprilTag);
    visionPortal = builder.build();
  }

  public Pose updateAprilTagPose() {
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    Pose newPose = null;
    tagFound = false;
    for (AprilTagDetection detection : currentDetections) {
      if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
        tagFound = true;
        double x = detection.robotPose.getPosition().x;
        double y = detection.robotPose.getPosition().y;
        double h_rad = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        Pose2D visionPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, h_rad);
        Pose pedroPose =
            PoseConverter.pose2DToPose(visionPose, InvertedFTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        double pose_x, pose_y;
        if (pedroPose.getX() < 0) {
          pose_x = -pedroPose.getX() + 72;
        } else {
          pose_x = 72 - pedroPose.getX();
        }
        if (pedroPose.getY() < 0) {
          pose_y = -pedroPose.getY() + 72;
        } else {
          pose_y = 72 - pedroPose.getY();
        }
        newPose = new Pose(pose_x, pose_y, h_rad);
        break;
      }
    }
    return newPose;
  }

  public void stopStreaming() {
    if (visionPortal != null) visionPortal.stopStreaming();
  }

  public boolean isTagFound() {
    return tagFound;
  }
}
