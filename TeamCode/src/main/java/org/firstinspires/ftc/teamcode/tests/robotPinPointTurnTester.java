package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
@TeleOp(name = "Robot Pinpoint Turn Tester", group = "Test")
// Comment this out to add to the OpMode list
public class robotPinPointTurnTester extends LinearOpMode{
    public static  double Kp = 0.025;
    public static double targetTurnAngle = 90;
    public static double min_power = 0.2;
    public static double max_power = 0.4;
    private boolean isTurnDone = false;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    GoBildaPinpointDriver pinpoint;

    @Override public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Drive Motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        isTurnDone = false;

        waitForStart();
        while(opModeIsActive()) {
            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();
            double currentAngle = pose2D.getHeading(AngleUnit.DEGREES);
            if (gamepad1.dpad_right) {
                telemetry.addData("Calling turn robot for ", currentAngle);
                telemetry.update();
                turnRobot();
            }
            telemetry.addData("Robot Final Angle", currentAngle);
            telemetry.addData("Robot Turn Target", targetTurnAngle);
            telemetry.update();
        }
    }
    public void turnRobot() {
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        double currentAngle = pose2D.getHeading(AngleUnit.DEGREES);
        double error = targetTurnAngle - currentAngle;

        telemetry.addData("Start: Robot Current Angle", currentAngle);
        telemetry.addData("Start: Robot Turn Target", targetTurnAngle);
        // Handle angle wrapping (e.g., turning from 170 to -170 degrees)
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        double power = error * Kp;
        // Clamp power to a reasonable range (e.g., -0.5 to 0.5) to prevent twitching
        power = Math.max(-max_power, Math.min(max_power, power));
        telemetry.addData("Start: Robot Turn Power", power);
        telemetry.update();
        sleep(1000);
        // Apply power (opposite for left/right)
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
        while (Math.abs(error) > 1) { // Loop until very close
            isTurnDone = false;
            pinpoint.update();
            pose2D = pinpoint.getPosition();
            currentAngle = pose2D.getHeading(AngleUnit.DEGREES);
//            currentAngle = follower.getHeading();
            error = targetTurnAngle - currentAngle;
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            power = error * Kp;
            telemetry.addData("Robot Current Angle", currentAngle);
            telemetry.addData("Robot Turn Target", targetTurnAngle);
            telemetry.addData("Robot Turn power", power);
            telemetry.update();
            if (power > 0 && power < min_power) {
                power = min_power;
            } else  if (power < 0 && power > -min_power) {
                power = -min_power;
            } else {
                power = Math.max(-max_power, Math.min(max_power, power));
            }
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(power);
        }
        isTurnDone = true;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
