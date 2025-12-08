package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intakeLaunch {

    private DcMotorEx intakeF, intakeM, shooter;
    private final Telemetry telemetry;
    public static double TRAIN_RPM_PERCENT = 1;
    public static double LAUNCH_VELOCITY = .7;
    public static int MAX_RPM = 1500;
    public static double INTAkE_TRANSFER_POWER = 1;

    public intakeLaunch(HardwareMap hardwareMap, Telemetry tel) {
        telemetry = tel;

        //Intake Motors
        intakeF = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeM = hardwareMap.get(DcMotorEx.class, "intakeMid");

        intakeF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        intakeF.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeM.setDirection(DcMotorSimple.Direction.REVERSE);

        //Shooter Motor
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // TODO: reverse motor directions if needed
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void runIntake(double intakePower, double transferPower) {
        telemetry.addData("Setting intake power to", intakePower);
        telemetry.addData("Setting transfer power to", transferPower);
        intakeF.setPower(intakePower);
        intakeM.setPower(transferPower);
        telemetry.addData("intake Front velocity ", intakeF.getVelocity());
        telemetry.update();
    }

    public void stopIntake() {
        telemetry.addData("Shutting down: ", "intake");
        telemetry.update();
        intakeM.setPower(0);
        intakeF.setPower(0);
    }

    public void takeShot(double launchPower, int waitTime) {
        telemetry.addData("Launching balls with power ", launchPower);
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        double targetVel = MAX_RPM * launchPower;
        while (runtime.milliseconds() < waitTime) {
            shooter.setVelocity(targetVel);
            // start feeding the ball as soon as launcher velocity reaches 90% of the target to reduce the transfer time
            if (Math.abs(shooter.getVelocity()) > MAX_RPM * (0.9 * launchPower) ) {
                telemetry.addData("start the intake during shot ", shooter.getVelocity());
                telemetry.update();
                intakeM.setPower(INTAkE_TRANSFER_POWER);
                intakeF.setPower(INTAkE_TRANSFER_POWER);
            } else {
                intakeM.setPower(0);
                intakeF.setPower(0.1);
            }
        }
    }

    public void powerOnLauncher(double launchPower) {
        telemetry.addData("Turning on Launcher with power ", launchPower);
        telemetry.update();
        double targetVel = MAX_RPM * launchPower * TRAIN_RPM_PERCENT;
        shooter.setVelocity(targetVel);
    }

    public void stopLauncher() {
        telemetry.addData("Shutting down: ", "Launcher");
        telemetry.update();
        shooter.setPower(0);
        intakeM.setPower(0);
        intakeF.setPower(0);
    }

    public void waitForMSeconds (int ms) {
        try {
            sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}

