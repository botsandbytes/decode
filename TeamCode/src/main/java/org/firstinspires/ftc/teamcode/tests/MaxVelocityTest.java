package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotorEx leftFront, leftBack, rightBack, rightFront;
        leftFront = hardwareMap.get(DcMotorEx.class, "shooter1");
        leftBack = hardwareMap.get(DcMotorEx.class, "shooter2");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            leftFront.setPower(1);
            leftBack.setPower(1);
            telemetry.addData("fl", leftFront.getVelocity());
            telemetry.addData("bl", leftBack.getVelocity());
            telemetry.update();
        }
    }
}