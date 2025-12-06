package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BBRobot {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx Motor_FL;
    private DcMotorEx Motor_FR;
    private DcMotorEx Motor_BR;
    private DcMotorEx Motor_BL;

    // Flag so other files can tell if this is TeleOp
    public boolean isTeleOp = false;

    public BBRobot(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;
        initHardware();
    }

    private void initHardware() {
        telemetry.addData("BB Robot", "Initializing Motors");
        telemetry.update();

        // Drive Motors
        Motor_FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        Motor_FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        Motor_BR = hardwareMap.get(DcMotorEx.class, "rightBack");
        Motor_BL = hardwareMap.get(DcMotorEx.class, "leftBack");

        // Motor directions
        Motor_FL.setDirection(DcMotor.Direction.FORWARD);
        Motor_BL.setDirection(DcMotor.Direction.FORWARD);
        Motor_FR.setDirection(DcMotor.Direction.REVERSE);
        Motor_BR.setDirection(DcMotor.Direction.REVERSE);

        // Zero-power behavior
        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use power directly
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();
    }

    /** Simple movement methods **/

    public void stopMotors() {
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }

    public void moveForward(double power) {
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);
    }

    public void moveBackward(double power) {
        Motor_FL.setPower(-power);
        Motor_FR.setPower(-power);
        Motor_BR.setPower(-power);
        Motor_BL.setPower(-power);
    }

    public void moveLeft(double power) {
        Motor_FL.setPower(power);
        Motor_FR.setPower(-power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(-power);
    }

    public void moveRight(double power) {
        Motor_FL.setPower(-power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(-power);
        Motor_BL.setPower(power);
    }

    /** Helper: CRServo power control (-1 to 1) **/
}