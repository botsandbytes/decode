package org.firstinspires.ftc.teamcode.tests;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Shooter Max Velocity Test", group="Test")
@Configurable
public class MaxVelocityTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static String name = "shooter";
    public static String name2 = "shooter2";
    public static Double power = 1.0;
    public static Double targetVelocity = 1350.0;
    public static PIDFCoefficients pidfCoefficients= new PIDFCoefficients(150, 0, 0, 22.8);
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx testMotor, testMotor2;
        boolean time_recorded = false;
        boolean time2_recorded = false;
        String timeTaken="0";
        String timeTaken2="0";
        double maxMotorVelocity = 0;
        double maxMotor2Velocity = 0;
        double motor_velocity = 0, motor2_velocity = 0, error = 0, error2 = 0;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        testMotor = hardwareMap.get(DcMotorEx.class, name);
//        testMotor2 = hardwareMap.get(DcMotorEx.class, name2);
//
//        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        testMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        testMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        testMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            testMotor = hardwareMap.get(DcMotorEx.class, name);
//          testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            testMotor2 = hardwareMap.get(DcMotorEx.class, name2);
//          testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            testMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            testMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            testMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            testMotor.setPower(power);
//            testMotor2.setPower(power);
            testMotor.setVelocity(targetVelocity);
            testMotor2.setVelocity(targetVelocity);
            motor_velocity = testMotor.getVelocity();
            motor2_velocity = testMotor2.getVelocity();
            error = targetVelocity - motor_velocity;
            error2 = targetVelocity - motor2_velocity;
            telemetryM.addData("Current Velocity", motor_velocity);
            telemetryM.addData("Current Velocity", motor2_velocity);
//            telemetryM.update();
            if (motor_velocity > (targetVelocity * .95) && !time_recorded) {
                maxMotorVelocity = motor_velocity;
                timeTaken = runtime.toString();
                time_recorded = true;
            }

            if (motor2_velocity > (targetVelocity * .95) && !time2_recorded) {
                maxMotor2Velocity = motor2_velocity;
                timeTaken2 = runtime.toString();
                time2_recorded = true;
            }
            telemetryM.addData("Motor Velocity", motor_velocity);
            telemetryM.addData("Motor Velocity Motor 2", motor2_velocity);
            telemetryM.addData("Motor error", error);
            telemetryM.addData("Motor error Motor 2", error2);
            telemetryM.addData("Max Velocity", maxMotorVelocity);
            telemetryM.addData("Max Velocity Motor 2", maxMotor2Velocity);
            telemetryM.addData("time Taken", timeTaken);
            telemetryM.addData("time Taken2", timeTaken2);
            telemetryM.update();

            if(gamepad1.x) {
                telemetryM.addLine("Stop the motors");
                telemetryM.update();
                testMotor.setPower(0);
                testMotor2.setPower(0);
            }
            if(gamepad1.y) {
                telemetryM.addLine("Resetting the values");
                telemetryM.update();
                maxMotorVelocity = 0;
                maxMotor2Velocity = 0;
                time_recorded = false;
                time2_recorded = false;
                timeTaken="0";
                timeTaken2="0";
            }
        }
    }
}