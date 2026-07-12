package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.utilities.ConfigLoader;

public class Intake {
    private final CachingDcMotorEx intakeFront;
    private final CachingDcMotorEx intakeMid;

    public Intake(HardwareMap hardwareMap) {
        intakeFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeFront"));
        intakeMid = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMid"));

        double tolerance = ConfigLoader.getDouble("caching.intake_tolerance");
        intakeFront.setCachingTolerance(tolerance);
        intakeMid.setCachingTolerance(tolerance);

        intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMid.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(double intakePower, double transferPower) {
        intakeFront.setPower(intakePower);
        intakeMid.setPower(transferPower);
    }

    public void stop() {
        intakeFront.setPower(0);
        intakeMid.setPower(0);
    }

    public boolean isRunning() {
        double tolerance = ConfigLoader.getDouble("caching.intake_tolerance");
        return Math.abs(intakeFront.getPower()) > tolerance || Math.abs(intakeMid.getPower()) > tolerance;
    }

    public Command runIntakeCommand(double intakePower, double transferPower) {
        return Command.build()
                .setStart(() -> run(intakePower, transferPower))
                .setEnd(interrupted -> stop())
                .requiring(this);
    }

    public Command stopIntakeCommand() {
        return Command.build()
                .setStart(this::stop)
                .requiring(this);
    }
}
