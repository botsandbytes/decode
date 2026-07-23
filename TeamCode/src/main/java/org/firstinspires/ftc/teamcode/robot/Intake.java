package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.robot.config.config;

public class Intake {
  private final CachingDcMotorEx intakeFront;
  private final CachingDcMotorEx intakeMid;

  public Intake(HardwareMap hardwareMap) {
    intakeFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeFront"));
    intakeMid = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMid"));

    double tolerance = config.caching.intake_tolerance;
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
    return Math.abs(intakeFront.getPower()) > 0.01 || Math.abs(intakeMid.getPower()) > 0.01;
  }

  public void periodic() {
    // Fulfills periodic contract, no-op for pure actuator
  }

  public Command runIntakeCommand(double intakePower, double transferPower) {
    return Command.build()
        .setStart(() -> run(intakePower, transferPower))
        .setEnd(interrupted -> stop())
        .requiring(this);
  }

  public Command stopIntakeCommand() {
    return Command.build().setStart(this::stop).requiring(this);
  }
}
