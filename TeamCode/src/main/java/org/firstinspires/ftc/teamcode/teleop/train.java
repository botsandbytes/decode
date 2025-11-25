package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;
import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// IMPORT THE PINPOINT DRIVER DIRECTLY
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

// FTC SDK NAVIGATION IMPORTS (No RoadRunner)
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D; // Standard SDK Pose
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// VISION IMPORTS
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Shooter Training (No RR - Direct Pinpoint)")
@Configurable
public class train extends OpMode {

    // --- Field Goal Location ---
    public static double GOAL_X = -62.9921259843;
    public static double GOAL_Y = -GOAL_X;

    // --- Training Params ---
    public static double TRAIN_HOOD_ANGLE = 0.5;
    public static double TRAIN_RPM_PERCENT = 0.5;
    public static int MAX_RPM = 3000;

    // --- Turret Servo (y = mx + b) ---
    public static double TURRET_M = 0.003;
    public static double TURRET_B = 0.5;

    // --- Motor Powers ---
    public static double INTAKE_L_POWER = 0.8;
    public static double INTAKE_M_POWER = 1.0;

    // --- Pinpoint Config ---
    public static double PINPOINT_X_OFFSET_MM = -170;
    public static double PINPOINT_Y_OFFSET_MM = 40;

    // =========================================================
    // HARDWARE
    // =========================================================

    // Camera Config
    private final Position cameraPosition = new Position(DistanceUnit.CM, -4, -18, 22, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    // Localization (Direct Driver)
    private GoBildaPinpointDriver odo;

    // Drive Motors (Raw Hardware)
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;

    // Mechanisms
    private DcMotorEx shooterMotor;
    private DcMotor intakeM, intakeL;
    private Servo turretServo, hoodServo;

    // Vision
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Start Pose (Using SDK Pose2D)
    private Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private boolean tagFound = false;

    // State
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private boolean isShooting = false;
    private boolean isIntaking = false;

    // Telemetry
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        try {
            // --- 1. INIT DRIVE MOTORS ---
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // TODO: Verify your motor directions here
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            // --- 2. INIT PINPOINT ---
            // Ensure your config name matches "odo" or change this string
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // Apply offsets (using MM as native unit for accuracy)
            odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);

            // Set Encoder resolution (Example: 4-Bar Pods)
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // Set Directions
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

            // Reset
            odo.resetPosAndIMU();

            // --- 3. INIT MECHANISMS ---
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            intakeM = hardwareMap.dcMotor.get("intakem");
            intakeL = hardwareMap.dcMotor.get("intakel");
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretServo = hardwareMap.get(Servo.class, "turret");
            hoodServo = hardwareMap.get(Servo.class, "hood");

        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }

        initAprilTag();
        telemetry.addData("Status", "Initialized. Scan Tag for Start Pose.");
    }

    @Override
    public void init_loop() {
        updateAprilTagPose();

        telemetry.addData("Tag Found", tagFound ? "YES" : "NO");
        telemetry.addData("Start X", "%.2f", startPose.getX(DistanceUnit.INCH));
        telemetry.addData("Start Y", "%.2f", startPose.getY(DistanceUnit.INCH));
        telemetry.addData("Start H", "%.2f", startPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        // --- SYNC POSE ---
        // Push the vision pose into the Pinpoint computer
        odo.setPosition(startPose);

        if (visionPortal != null) visionPortal.stopStreaming();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // 1. UPDATE LOCALIZATION
        odo.update();
        Pose2D currentPose = odo.getPosition();

        // 2. DRIVE CONTROL (Manual Mecanum Math)
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.pow(Math.abs(y) + Math.abs(x) + Math.abs(rx), 3), 1);
        leftFront.setPower(Math.pow(y + x + rx, 3) / denominator);
        leftBack.setPower(Math.pow(y - x + rx, 3) / denominator);
        rightFront.setPower(Math.pow(y - x - rx, 3) / denominator);
        rightBack.setPower(Math.pow(y + x - rx, 3) / denominator);

        // 3. SHOOTER & TURRET
        // Extract values from SDK Pose2D
        double curX = currentPose.getX(DistanceUnit.INCH);
        double curY = currentPose.getY(DistanceUnit.INCH);
        double curH_Rad = currentPose.getHeading(AngleUnit.RADIANS);

        double dist = Math.hypot(GOAL_X - curX, GOAL_Y - curY);

        double angleToGoalRad = Math.atan2(GOAL_Y - curY, GOAL_X - curX);
        double relativeAngleDeg = Math.toDegrees(angleToGoalRad - curH_Rad);

        // Normalize
        while (relativeAngleDeg > 180) relativeAngleDeg -= 360;
        while (relativeAngleDeg <= -180) relativeAngleDeg += 360;

        // Clamp

        // --- TURRET (Commented Out) ---
        // double servoPos = (clampedAngle * TURRET_M) + TURRET_B;
        // turretServo.setPosition(Math.max(0, Math.min(1, servoPos)));

        hoodServo.setPosition(TRAIN_HOOD_ANGLE);

        // 4. MECHANISMS
        handleIntakeShooter();

        // 5. TELEMETRY
        telemetryM.addData("Dist", dist);
        telemetryM.addData("Turret Ang", relativeAngleDeg);
        // telemetryM.addData("Servo", servoPos);

        telemetryM.addData("Pose", String.format("%.1f, %.1f, %.1f°",
            curX, curY, Math.toDegrees(curH_Rad)));

        telemetryM.update(telemetry);
    }

    private void handleIntakeShooter() {
        if (gamepad2.a) { // INTAKE
            isIntaking = true;
            isShooting = false;
        } else if (gamepad2.x) { // SHOOT
            isShooting = true;
            isIntaking = false;
            shooterTimer.reset();
        } else { // IDLE
            isIntaking = false;
            if (!isShooting) {
                shooterMotor.setVelocity(0);
                intakeM.setPower(0);
                intakeL.setPower(0);
            }
        }

        if (isIntaking) {
            intakeL.setPower(INTAKE_L_POWER);
            intakeM.setPower(0);
            shooterMotor.setVelocity(0);
        }

        if (isShooting) {
            double targetVel = MAX_RPM * TRAIN_RPM_PERCENT;
            shooterMotor.setVelocity(targetVel);

            if (shooterTimer.seconds() > 0.5) {
                intakeM.setPower(INTAKE_M_POWER);
            } else {
                intakeM.setPower(0);
            }

            if (shooterTimer.seconds() > 3.0) {
                isShooting = false;
            }
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder().setCameraResolution(new Size(1920, 1080)).setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void updateAprilTagPose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                tagFound = true;

                // Convert Vision Pose to SDK Pose2D
                double x = detection.robotPose.getPosition().x;
                double y = detection.robotPose.getPosition().y;
                double h_deg = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                startPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h_deg);
                break;
            }
        }
    }
}