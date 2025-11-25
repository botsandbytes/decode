package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;
import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

// --- FIELD IMPORTS ---
import com.bylazar.field.PanelsField;
import com.bylazar.field.FieldManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// IMPORT THE PINPOINT DRIVER DIRECTLY
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

// FTC SDK NAVIGATION IMPORTS
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// VISION IMPORTS
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Shooter Training")
@Configurable
public class train extends OpMode {

    // --- Field Goal Location ---
    public static double GOAL_X = -62.9921259843;
    public static double GOAL_Y = -GOAL_X;

    // --- Training Params ---
    public static double TRAIN_HOOD_ANGLE = 0.5;
    public static double TRAIN_RPM_PERCENT = 0.5;
    public static int MAX_RPM = 1560;


    // --- Motor Powers ---
    public static double INTAKE_L_POWER = 1.0;
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
    private Servo hoodServo;

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

    // --- FIELD VISUALIZER ---
    private FieldManager field;

    @Override
    public void init() {
        try {
            // --- FIELD SETUP ---
            field = PanelsField.INSTANCE.getField();
            field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());

            // --- 1. INIT DRIVE MOTORS ---
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // --- 2. INIT PINPOINT ---
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.resetPosAndIMU();

            // --- 3. INIT MECHANISMS ---
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            intakeM = hardwareMap.dcMotor.get("intakem");
            intakeL = hardwareMap.dcMotor.get("intakel");
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        odo.setPosition(startPose);
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // 1. UPDATE LOCALIZATION
        odo.update();
        Pose2D currentPose = odo.getPosition();

        // 2. DRIVE CONTROL
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.pow(Math.abs(y) + Math.abs(x) + Math.abs(rx), 3), 1);
        leftFront.setPower(Math.pow(y + x + rx, 3) / denominator);
        leftBack.setPower(Math.pow(y - x + rx, 3) / denominator);
        rightFront.setPower(Math.pow(y - x - rx, 3) / denominator);
        rightBack.setPower(Math.pow(y + x - rx, 3) / denominator);

        // 3. SHOOTER & TURRET
        double curX = currentPose.getX(DistanceUnit.INCH);
        double curY = currentPose.getY(DistanceUnit.INCH);
        double curH_Rad = currentPose.getHeading(AngleUnit.RADIANS);

        double dist = Math.hypot(GOAL_X - curX, GOAL_Y - curY);

        double angleToGoalRad = Math.atan2(GOAL_Y - curY, GOAL_X - curX);
        double relativeAngleDeg = Math.toDegrees(angleToGoalRad - curH_Rad);

        while (relativeAngleDeg > 180) relativeAngleDeg -= 360;
        while (relativeAngleDeg <= -180) relativeAngleDeg += 360;

        hoodServo.setPosition(TRAIN_HOOD_ANGLE);

        // 4. DRAW FIELD
        drawRobotOnField(curX, curY, curH_Rad);

        // 5. MECHANISMS & TELEMETRY
        handleIntakeShooter();

        telemetryM.addData("Dist", dist);
        telemetryM.addData("Turret Ang", relativeAngleDeg);
        telemetryM.addData("Pose", String.format("%.1f, %.1f, %.1f°",
            curX, curY, Math.toDegrees(curH_Rad)));
        telemetryM.addData("shooter V", Math.abs(shooterMotor.getVelocity()));
        telemetryM.addData("Shooter Target", MAX_RPM * 0.9 * TRAIN_RPM_PERCENT);
        telemetryM.addData("Shooter Minimum", MAX_RPM * 0.85 * TRAIN_RPM_PERCENT);
        telemetryM.update(telemetry);
    }

    /**
     * Draws the robot, heading, and goal line on the Panels dashboard.
     * UPDATED: Explicitly moves cursor between line segments to ensure a square is drawn.
     */
    private void drawRobotOnField(double x, double y, double h) {
        h += Math.PI / 2;
        // --- 1. Draw Line to Goal (Cyan) ---
        field.setStyle("none", "cyan", 2.0);
        field.moveCursor(x, y);
        field.line(GOAL_X, GOAL_Y);

        // --- 2. Calculate Corners (18x18 inches) ---
        // Formula: x' = x*cos(h) - y*sin(h)
        //          y' = x*sin(h) + y*cos(h)
        // FL (9, 9), FR (9, -9), BR (-9, -9), BL (-9, 9)

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

        // We explicit move cursor to the start of each segment to prevent "fan" artifacts
        field.moveCursor(xFL, yFL);
        field.line(xFR, yFR);

        field.moveCursor(xFR, yFR);
        field.line(xBR, yBR);

        field.moveCursor(xBR, yBR);
        field.line(xBL, yBL);

        field.moveCursor(xBL, yBL);
        field.line(xFL, yFL); // Close the box

        // --- 4. Draw Heading Line (Yellow) ---
        // From center (x,y) to the midpoint of the Front Face (9, 0 local)
        // This ensures the line is exactly perpendicular to the front edge of the box.
        double frontX = x + (halfSize * cosA);
        double frontY = y + (halfSize * sinA);

        field.setStyle("none", "yellow", 2.0);
        field.moveCursor(x, y);
        field.line(frontX, frontY);

        // --- 5. Push Update ---
        field.update();
    }

    private void handleIntakeShooter() {
        if (gamepad1.a) { // INTAKE
            isIntaking = true;
            isShooting = false;
        } else if (gamepad1.x) { // SHOOT
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
            double targetVel = MAX_RPM * 0.9 * TRAIN_RPM_PERCENT;
            shooterMotor.setVelocity(targetVel);

            if (Math.abs(shooterMotor.getVelocity()) > MAX_RPM * 0.85 * TRAIN_RPM_PERCENT) {
                intakeM.setPower(INTAKE_M_POWER);
            } else {
                intakeM.setPower(0);
            }

            if (gamepad1.y) {
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
                double x = detection.robotPose.getPosition().x;
                double y = detection.robotPose.getPosition().y;
                double h_deg = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                startPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h_deg);
                break;
            }
        }
    }
}