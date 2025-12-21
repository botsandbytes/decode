package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Lumos;

@TeleOp(name = "light", group = "Test")
public class light extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lumos lumos = new Lumos(hardwareMap);
        int a = 0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
                    lumos.red();
                    telemetry.addData("pos", lumos.getPosition());
        }
    }
}