package org.firstinspires.ftc.teamcode.utilities;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionUtil {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean tagFound = false;
    private Pose2D lastKnownPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    // Camera Config
    private final Position cameraPosition = new Position(DistanceUnit.CM, 0, -20, 17, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    public void initAprilTag(HardwareMap hardwareMap, boolean useWebcam) {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder()
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

    public Pose2D updateAprilTagPose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        tagFound = false;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                tagFound = true;
                double x = detection.robotPose.getPosition().x;
                double y = detection.robotPose.getPosition().y;
                double h_deg = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                lastKnownPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h_deg);
                break;
            }
        }
        return lastKnownPose;
    }

    public void stopStreaming() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    public boolean isTagFound() {
        return tagFound;
    }
    
    public Pose2D getLastKnownPose() {
        return lastKnownPose;
    }
}
