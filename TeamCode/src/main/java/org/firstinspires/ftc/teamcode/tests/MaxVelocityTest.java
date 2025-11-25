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
        leftFront = hardwareMap.get(DcMotorEx.class, "shooter");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            leftFront.setPower(1);
            telemetry.addData("fl", leftFront.getVelocity());
            telemetry.update();
        }
    }
}