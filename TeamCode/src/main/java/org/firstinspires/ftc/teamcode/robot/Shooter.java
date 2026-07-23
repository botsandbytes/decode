package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.LaunchParameters;
import org.firstinspires.ftc.teamcode.robot.config.config;

@Configurable
public class Shooter {

  private final CachingDcMotorEx shooter1;
  private final CachingDcMotorEx shooter2;
  private final CachingServo hood;

  public Shooter(HardwareMap hardwareMap) {
    shooter1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter"));
    shooter2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter2"));
    hood = new CachingServo(hardwareMap.get(Servo.class, "hood"));

    shooter1.setCachingTolerance(config.caching.flywheel_tolerance);
    shooter2.setCachingTolerance(config.caching.flywheel_tolerance);
    hood.setCachingTolerance(config.caching.hood_tolerance);

    shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

    shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

    setShooterPIDFCoefficients();
    setHoodPosition(0);
  }

  public final void setShooterPIDFCoefficients() {
    shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.shooter.pidf);
    shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.shooter.pidf);
  }

  public double getShooterVelocity() {
    return shooter1.getVelocity();
  }

  public double getShooterPower() {
    return shooter1.getPower();
  }

  public void stop() {
    shooter1.setPower(0);
    shooter2.setPower(0);
  }

  private void powerOnLauncher(double power) {
    double targetVel = config.shooter.max_rpm * power;
    shooter1.setVelocity(targetVel);
    shooter2.setVelocity(targetVel);
  }

  public final void setHoodPosition(double position) {
    hood.setPosition(position);
  }

  public void setHoodLongShotPosition() {
    hood.setPosition(0);
  }

  public LaunchParameters calculateLaunchParameters(
      Pose currentPose, double goalX, double goalY, Alliance alliance) {
    double deltaX = goalX - currentPose.getX();
    double deltaY = goalY - currentPose.getY();
    double distance = Math.hypot(deltaX, deltaY);
    double angleRadians = Math.atan2(deltaY, deltaX);

    var lp = config.shooter.launch_params;
    double launchPower;
    double waitTime;

    if (distance <= lp.threshold_distance) {
      launchPower = lp.near.launch_power;
      waitTime = lp.near.wait_time;
    } else {
      double farBasePower =
          alliance == Alliance.RED ? lp.far.red_base_power : lp.far.blue_base_power;
      launchPower = farBasePower + ((distance - lp.threshold_distance) / lp.far.power_scale);
      waitTime = distance * lp.far.wait_time_scale;
    }

    return new LaunchParameters(launchPower, waitTime, Math.toDegrees(angleRadians));
  }

  private double targetPower = 0.0;

  public void setTargetPower(double power) {
    this.targetPower = power;
  }

  public double getTargetPower() {
    return targetPower;
  }

  public void periodic() {
    if (targetPower == 0) {
      stop();
    } else {
      powerOnLauncher(targetPower);
    }
  }

  public Command shootCommand(double power) {
    return Command.build()
        .setStart(() -> setTargetPower(power))
        .setEnd(interrupted -> setTargetPower(0))
        .requiring(this);
  }
}
