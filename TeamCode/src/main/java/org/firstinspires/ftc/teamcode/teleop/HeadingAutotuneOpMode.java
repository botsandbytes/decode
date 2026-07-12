package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.PIDAutotuner;

import java.util.List;

@TeleOp(name = "Chassis Heading Autotune", group = "Tuning")
public class HeadingAutotuneOpMode extends OpMode {

    private Follower follower;
    private PIDAutotuner autotuner;
    private TelemetryManager telemetryM;
    private List<LynxModule> allHubs;

    private boolean autotuneStarted = false;
    private double targetHeading = 0.0;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        FieldManager field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        autotuner = new PIDAutotuner();

        telemetryM.addLine("=== CHASSIS HEADING AUTOTUNE ===");
        telemetryM.addLine("Press [A] to start heading autotune");
        telemetryM.addLine("Robot chassis will oscillate left/right");
        telemetryM.addLine("");
        telemetryM.addLine("Make sure robot has space to rotate!");
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        follower.update();
        double currentHeading = follower.getHeading();

        if (gamepad1.a && !autotuneStarted) {
            targetHeading = currentHeading;
            // Oscillate around current heading with 0.25 turn power
            autotuner.startAutotune(currentHeading, targetHeading, 0.25);
            autotuneStarted = true;
        }

        if (autotuner.isRunning()) {
            double power = autotuner.updateAutotune(currentHeading);
            
            // Set rotation power (forward = 0, strafe = 0, turn = power)
            follower.setTeleOpDrive(0.0, 0.0, power, true);

            updateRunningTelemetry(currentHeading);

        } else if (autotuner.isComplete()) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
            updateCompleteTelemetry();

        } else if (autotuner.isFailed()) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
            updateFailedTelemetry();

        } else {
            updateIdleTelemetry(currentHeading);
        }

        telemetryM.update();
    }

    private void updateIdleTelemetry(double currentHeading) {
        telemetryM.addLine("=== READY ===");
        telemetryM.addData("Current Heading (Rad)", currentHeading);
        telemetryM.addData("Current Heading (Deg)", Math.toDegrees(currentHeading));
        telemetryM.addLine("");
        telemetryM.addLine("Press [A] to start heading autotune");
    }

    private void updateRunningTelemetry(double currentHeading) {
        telemetryM.addLine("=== AUTOTUNING CHASSIS HEADING... ===");
        telemetryM.addData("Target Heading (Rad)", targetHeading);
        telemetryM.addData("Current Heading (Rad)", currentHeading);
        telemetryM.addData("Crossings", autotuner.getCrossingCount());
        telemetryM.addLine("");
        telemetryM.addLine("Wait for oscillations...");
    }

    @SuppressLint("DefaultLocale")
    private void updateCompleteTelemetry() {
        telemetryM.addLine("=== COMPLETE ===");
        telemetryM.addLine("");
        telemetryM.addLine("--- TYREUS-LUYBEN (RECOMMENDED) ---");
        telemetryM.addLine("Good balance for smooth rotation:");
        telemetryM.addData("Kp_TL", autotuner.getKp_TL());
        telemetryM.addData("Ki_TL", autotuner.getKi_TL());
        telemetryM.addData("Kd_TL", autotuner.getKd_TL());
        telemetryM.addLine("");
        telemetryM.addLine("--- PESSEN INTEGRATION (STRONG HOLD) ---");
        telemetryM.addLine("Minimizes angle errors aggressively:");
        telemetryM.addData("Kp_PE", autotuner.getKp_Pessen());
        telemetryM.addData("Ki_PE", autotuner.getKi_Pessen());
        telemetryM.addData("Kd_PE", autotuner.getKd_Pessen());
        telemetryM.addLine("");
        telemetryM.addLine("--- ZIEGLER-NICHOLS (AGGRESSIVE) ---");
        telemetryM.addData("Kp_ZN", autotuner.getKp());
        telemetryM.addData("Ki_ZN", autotuner.getKi());
        telemetryM.addData("Kd_ZN", autotuner.getKd());
        telemetryM.addLine("");
        telemetryM.addData("Ku", autotuner.getKu());
        telemetryM.addData("Pu", autotuner.getPu());
    }

    private void updateFailedTelemetry() {
        telemetryM.addLine("=== FAILED ===");
        telemetryM.addLine("");
        telemetryM.addLine("Not enough oscillations (timeout)");
        telemetryM.addLine("Ensure robot can turn freely and wheels don't slip too much.");
        telemetryM.addLine("");
        telemetryM.addLine("Press [A] to retry");
        autotuneStarted = false;
    }
}
