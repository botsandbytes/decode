package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Intake Ball Tracker", group="Utilities")
public class distanceSensor extends LinearOpMode {

    private DistanceSensor intakeSensor;
//    private DcMotorEx intakeFront;
//    private DcMotorEx intakeMid;

    private static final int MAX_BALLS = 3;
    private static final double DISTANCE_THRESHOLD = 8.5; // cm, make it slightly shorter than funnel length
    private static final double STUCK_TIME = 3.0;        // (tune) seconds a ball sits in front of sensor before killing intake

    private int ballCount = 0;
    private boolean ballDetectedPreviously = false;
    private boolean killIntake = false;
    private double firstTimeBallDetected = -1;

    @Override
    public void runOpMode() {

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeD");
//        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
//        intakeMid = hardwareMap.get(DcMotorEx.class, "intakeMid");

        // start intake motors for testing purposes
//        intakeFront.setPower(1.0);
//        intakeMid.setPower(1.0);

        waitForStart();

        while (opModeIsActive()) {

            double distance = intakeSensor.getDistance(DistanceUnit.CM);


            if (distance < DISTANCE_THRESHOLD) {

                // first time seeing this ball
                if (firstTimeBallDetected == -1) {
                    firstTimeBallDetected = getRuntime();
                }

                // ball stuck or intake full then kill motors
                if (getRuntime() - firstTimeBallDetected >= STUCK_TIME) {
//                    intakeFront.setPower(0); //ask abt either setting to 0 or using the zeropowerbehavior function
//                    intakeMid.setPower(0);
                    killIntake = true;
                }

                // count the ball once as it passes
                if (!ballDetectedPreviously && ballCount < MAX_BALLS) {
                    ballCount++;
                    ballDetectedPreviously = true;
                }

            } else {
                // no ball detected
                ballDetectedPreviously = false;
                firstTimeBallDetected = -1;
            }

            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.addData("Ball Count", ballCount);
//            telemetry.addData("Intake Front Power", intakeFront.getPower());
//            telemetry.addData("Intake Mid Power", intakeMid.getPower());
            telemetry.addData("Intake", killIntake);
            telemetry.update();
        }
    }
}
