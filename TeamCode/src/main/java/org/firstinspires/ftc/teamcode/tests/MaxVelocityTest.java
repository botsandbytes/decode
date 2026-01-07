package org.firstinspires.ftc.teamcode.tests;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Max Velocity Test", group="Test")
@Configurable
public class MaxVelocityTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static String name = "shooter";
    public static String name2 = "shooter2";
    public static Double power = 0.6;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx testMotor, testMotor2;
        boolean time_recorded = false;
        boolean time2_recorded = false;
        String timeTaken="0";
        String timeTaken2="0";

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double maxMotorVelocity = 0;
            double maxMotor2Velocity = 0;
            testMotor = hardwareMap.get(DcMotorEx.class, name);
            testMotor2 = hardwareMap.get(DcMotorEx.class, name2);
            testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            testMotor.setPower(power);
            testMotor2.setPower(power);
            double motor_velocity = testMotor.getVelocity();
            double motor2_velocity = testMotor2.getVelocity();
            telemetry.addData("Current Velocity", motor_velocity);
            telemetry.addData("Current Velocity", motor2_velocity);
            telemetry.update();
            if (motor_velocity > 1400 && !time_recorded) {
                maxMotorVelocity = motor_velocity;
                timeTaken = runtime.toString();
                time_recorded = true;
            }

            if (motor2_velocity > 1400 && !time2_recorded) {
                maxMotor2Velocity = motor2_velocity;
                timeTaken2 = runtime.toString();
                time2_recorded = true;
            }
            telemetry.addData("Max Velocity", maxMotorVelocity);
            telemetry.addData("Max Velocity Motor 2", maxMotor2Velocity);
            telemetry.addData("time Taken", timeTaken);
            telemetry.addData("time Taken2", timeTaken2);
            telemetry.update();
        }
    }
}