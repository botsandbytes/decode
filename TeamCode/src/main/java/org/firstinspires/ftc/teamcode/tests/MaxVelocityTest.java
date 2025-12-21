package org.firstinspires.ftc.teamcode.tests;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Max Velocity Test", group="Test")
@Configurable
public class MaxVelocityTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static String name = "";
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront, leftBack, rightBack, rightFront;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double maxMotorVelocity = 0;
            String timeTaken="0";
            leftFront = hardwareMap.get(DcMotorEx.class, name);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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