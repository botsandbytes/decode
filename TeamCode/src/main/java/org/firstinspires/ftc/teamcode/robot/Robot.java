package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {

    // Drive Motors
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    // Mechanisms
    public DcMotorEx shooterMotor;
    public DcMotor intakeM, intakeL;
    public Servo hoodServo;

    // Localization
    // Constants
    public static double PINPOINT_X_OFFSET_MM = -177;
    public static double PINPOINT_Y_OFFSET_MM = 40;

    public Robot(HardwareMap hardwareMap) {
        initDrive(hardwareMap);
        initMechanisms(hardwareMap);
    }

    private void initDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
    }


    private void initMechanisms(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeM = hardwareMap.dcMotor.get("intakeMid");
        intakeL = hardwareMap.dcMotor.get("intakeFront");

        intakeL.setDirection(DcMotorEx.Direction.REVERSE);
        intakeM.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        hoodServo = hardwareMap.get(Servo.class, "hood");
    }

    public void setDrivePowers(double y, double x, double rx) {
        double denominator = Math.max(Math.pow(Math.abs(y) + Math.abs(x) + Math.abs(rx), 3), 1);
        leftFront.setPower(Math.pow(y + x + rx, 3) / denominator);
        leftBack.setPower(Math.pow(y - x + rx, 3) / denominator);
        rightFront.setPower(Math.pow(y - x - rx, 3) / denominator);
        rightBack.setPower(Math.pow(y + x - rx, 3) / denominator);
    }
    public void setShooterVelocity(double velocity) {
        shooterMotor.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        return shooterMotor.getVelocity();
    }

    public void setIntakePower(double power) {
        intakeL.setPower(power);
        intakeM.setPower(power);
    }
    
    public void setIntakeFrontPower(double power) {
        intakeL.setPower(power);
    }

    public void setIntakeMidPower(double power) {
        intakeM.setPower(power);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }
}
