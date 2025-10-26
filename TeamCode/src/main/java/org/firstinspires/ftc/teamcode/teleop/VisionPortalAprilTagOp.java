package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import android.util.Size;
import java.util.List;

@TeleOp
public class VisionPortalAprilTagOp extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public void runOpMode(){

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)       // overlay tag IDs on preview
                .setDrawTagOutline(true)  // overlay tag outlines on preview
                .build();

        //aprilTag.setDecimation(2); needed?

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam"); // change webcam name
        visionPortal = new VisionPortal.Builder()
                .setCamera(cam)                                   // attach AprilTag processor
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))         // preview resolution
                .enableLiveView(true)                             // show live preview
                .setAutoStopLiveView(true)                        // stop preview when OpMode ends
                .build();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // get list of detected tags
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections == null || detections.isEmpty()) {
                telemetry.addLine("No AprilTags detected");
            } else {
                telemetry.addData("Detected Tags", detections.size());

                // print details for each tag
                for (AprilTagDetection tag : detections) {
                    telemetry.addData("Tag ID", tag.id);

                    // Fixed telemetry line
                    if (tag.center != null) {
                        telemetry.addData("Center (px)", "%.1f, %.1f", tag.center.x, tag.center.y);
                    } else {
                        telemetry.addData("Center (px)", "unknown");
                    }

                    Object ftcPose = tag.ftcPose;
                    if (ftcPose != null) {
                        try {
                            double x = ftcPose.getClass().getField("x").getDouble(ftcPose);
                            double y = ftcPose.getClass().getField("y").getDouble(ftcPose);
                            double z = ftcPose.getClass().getField("z").getDouble(ftcPose);
                            double yaw = ftcPose.getClass().getField("yaw").getDouble(ftcPose);
                            double range = ftcPose.getClass().getField("range").getDouble(ftcPose);

                            telemetry.addData("Pose XYZ (m)", String.format("%.3f, %.3f, %.3f", x, y, z));
                            telemetry.addData("Yaw (deg) / Range (m)", String.format("%.1f / %.3f", Math.toDegrees(yaw), range));

                            // Distance from robot to AprilTag
                            telemetry.addData("Distance to tag (m)", String.format("%.3f", range));

                        } catch (Exception e) {
                            telemetry.addData("Pose read", "failed: " + e.getClass().getSimpleName());
                        }
                    } else {
                        telemetry.addData("Pose", "not available");
                        telemetry.addData("Distance to tag", "not available");
                    }
                }
            }
            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }
}
