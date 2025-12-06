package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotorEx leftFront, leftBack, rightBack, rightFront;
        double maxMotorVelocity = 0;
        String timeTaken="0";
        leftFront = hardwareMap.get(DcMotorEx.class, "shooter");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            leftFront.setPower(1);
            double motor_velocity = leftFront.getVelocity();
            telemetry.addData("Current Velocity", motor_velocity);
            telemetry.update();
            if (motor_velocity > maxMotorVelocity) {
                maxMotorVelocity = motor_velocity;
                timeTaken = runtime.toString();
            }
            telemetry.addData("Max Velocity", maxMotorVelocity);
            telemetry.addData("time Taken", timeTaken);
            telemetry.update();

        }
    }
}