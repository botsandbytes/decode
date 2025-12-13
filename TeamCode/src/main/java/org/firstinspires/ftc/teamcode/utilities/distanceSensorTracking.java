package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Intake Ball Tracker", group="Utilities")
public class distanceSensorTracking extends LinearOpMode {

    private DistanceSensor intakeSensor;

    private static final int MAX_BALLS = 3;
    private static final double DISTANCE_THRESHOLD = 8.5;
    private static final double STUCK_TIME = .500;

    private int ballCount = 0;
    private double[] ballDistances = new double[MAX_BALLS];
    private boolean ballDetectedPreviously = false;
    private double firstTimeBallDetected = -1;
    private boolean intakeFull = false;

    @Override
    public void runOpMode() {

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeD");

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double distance = intakeSensor.getDistance(DistanceUnit.CM);

            if (distance < DISTANCE_THRESHOLD && !intakeFull) {

                if (firstTimeBallDetected == -1) {
                    firstTimeBallDetected = getRuntime();
                }

                if (getRuntime() - firstTimeBallDetected >= STUCK_TIME) {
                    intakeFull = true;
                    ballCount = MAX_BALLS;
                }

                if (!ballDetectedPreviously && ballCount < MAX_BALLS) {
                    ballDistances[ballCount] = distance;
                    ballCount++;
                    ballDetectedPreviously = true;
                }

            } else {
                ballDetectedPreviously = false;
                firstTimeBallDetected = -1;
            }

            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Intake Full", intakeFull);

            for (int i = 0; i < ballCount && i < MAX_BALLS; i++) {
                telemetry.addData("Ball " + (i + 1) + " Distance", "%.2f cm", ballDistances[i]);
            }

            telemetry.update();
        }
    }
}
