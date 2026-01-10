package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Distance Sensor Test", group="Tests")
public class distanceSensorTest extends LinearOpMode {

    private DistanceSensor intakeSensor;

    @Override
    public void runOpMode() {

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeD");

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double distance = intakeSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.update();
        }
    }
}
