// ============================================================
// OPMODE
// ============================================================

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
import org.firstinspires.ftc.teamcode.robot.IntakeLauncher;
import org.firstinspires.ftc.teamcode.robot.TurretAutotuner;

import java.util.List;

@TeleOp(name = "Turret Autotune", group = "Tuning")
public class TurretAutotuneOpMode extends OpMode {

    private IntakeLauncher launcher;
    private TurretAutotuner autotuner;
    private Follower follower;
    private TelemetryManager telemetryM;
    private List<LynxModule> allHubs;

    private boolean autotuneStarted = false;

    @Override
    public void init() {
        // Initialize hardware
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        FieldManager field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        launcher = new IntakeLauncher(hardwareMap, telemetry, follower);
        autotuner = new TurretAutotuner();

        telemetryM.addLine("=== TURRET AUTOTUNE ===");
        telemetryM.addLine("Press [A] to start");
        telemetryM.addLine("Turret will oscillate for ~15s");
        telemetryM.addLine("");
        telemetryM.addLine("Make sure turret can move freely!");
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Clear bulk cache
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        follower.update();

        // Start autotune
        if (gamepad1.a && !autotuneStarted) {
            double currentAngle = launcher.getCurrentTurnAngle();
            autotuner.startAutotune(currentAngle, 0.3);
            autotuneStarted = true;
        }

        // Run autotune
        if (autotuner.isRunning()) {
            double currentAngle = launcher.getCurrentTurnAngle();
            double power = autotuner.updateAutotune(currentAngle);

            // Use IntakeLauncher's method to avoid hardware conflicts
            launcher.setTurretPowerRaw(power);

            updateRunningTelemetry(currentAngle);

        } else if (autotuner.isComplete()) {
            launcher.setTurretPowerRaw(0);
            updateCompleteTelemetry();

        } else if (autotuner.isFailed()) {
            launcher.setTurretPowerRaw(0);
            updateFailedTelemetry();

        } else {
            updateIdleTelemetry();
        }

        telemetryM.update();
    }

    private void updateIdleTelemetry() {
        telemetryM.addLine("=== READY ===");
        telemetryM.addData("Current Turret Angle", launcher.getCurrentTurnAngle());
        telemetryM.addLine("");
        telemetryM.addLine("Press [A] to start autotune");
    }

    private void updateRunningTelemetry(double currentAngle) {
        telemetryM.addLine("=== AUTOTUNING... ===");
        telemetryM.addData("Current Angle", currentAngle);
        telemetryM.addData("Crossings", autotuner.getCrossingCount());
        telemetryM.addLine("");
        telemetryM.addLine("Wait for oscillations...");
    }

    @SuppressLint("DefaultLocale")
    private void updateCompleteTelemetry() {
        telemetryM.addLine("=== COMPLETE ===");
        telemetryM.addLine("");
        telemetryM.addLine("--- TYREUS-LUYBEN (RECOMMENDED) ---");
        telemetryM.addLine("No overshoot, best for turret aiming:");
        telemetryM.addData("Kp_TL", autotuner.getKp_TL());
        telemetryM.addData("Ki_TL", autotuner.getKi_TL());
        telemetryM.addData("Kd_TL", autotuner.getKd_TL());
        telemetryM.addLine("");
        telemetryM.addLine("--- ZIEGLER-NICHOLS (AGGRESSIVE) ---");
        telemetryM.addLine("Faster but may overshoot:");
        telemetryM.addData("Kp_ZN", autotuner.getKp());
        telemetryM.addData("Ki_ZN", autotuner.getKi());
        telemetryM.addData("Kd_ZN", autotuner.getKd());
        telemetryM.addLine("");
        telemetryM.addData("Ku", autotuner.getKu());
        telemetryM.addData("Pu", autotuner.getPu());
        telemetryM.addLine("");
        telemetryM.addLine("RECOMMENDED (Tyreus-Luyben):");
        telemetryM.addLine(String.format(
            "PIDFCoefficients pidfCoefficients = new PIDFCoefficients(%.4f, %.6f, %.6f, 0);",
            autotuner.getKp_TL(), autotuner.getKi_TL(), autotuner.getKd_TL()
        ));
        telemetryM.addLine("Keep: double f = 0.08;");
    }

    private void updateFailedTelemetry() {
        telemetryM.addLine("=== FAILED ===");
        telemetryM.addLine("");
        telemetryM.addLine("Not enough oscillations");
        telemetryM.addLine("Check:");
        telemetryM.addLine("- Turret can move freely?");
        telemetryM.addLine("- Increase relay power?");
        telemetryM.addLine("");
        telemetryM.addLine("Press [A] to retry");
        autotuneStarted = false;
    }
}