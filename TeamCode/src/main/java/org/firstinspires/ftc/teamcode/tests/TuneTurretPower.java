package org.firstinspires.ftc.teamcode.tests;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.IntakeLauncher;

import java.util.List;

@TeleOp(name = "Tune Turret Power", group = "Test")
@Configurable
public class TuneTurretPower extends LinearOpMode {
    public static double min = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Follower follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();
        IntakeLauncher intakeLauncher = new IntakeLauncher(hardwareMap, telemetry, follower);



        if (isStopRequested()) return;

        while (opModeIsActive()) {
            intakeLauncher.tuneMinTurretPower(min);
        }
    }
}