package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.LaunchParameters;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@Configurable
public class Shooter {

  private final DcMotorEx shooter1;
  private final DcMotorEx shooter2;
  private final Servo hood;

  public Shooter(HardwareMap hardwareMap) {
    shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
    shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
    hood = hardwareMap.get(Servo.class, "hood");

    shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

    shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

    setShooterPIDFCoefficients();
    setHoodShortShotPosition();
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

  public final void setHoodShortShotPosition() {
    hood.setPosition(config.shooter.hood.low_position);
  }

  public final void setHoodLongShotPosition() {
    hood.setPosition(config.shooter.hood.high_position);
  }

  public LaunchParameters calculateLaunchParameters(
      Pose currentPose, double goalX, double goalY, Alliance alliance) {
    double distanceXToGoal = goalX - currentPose.getX();
    double distanceYToGoal = goalY - currentPose.getY();
    double distanceToGoal = Math.hypot(distanceXToGoal, distanceYToGoal);
    double launchAngleRadians = Math.atan2(distanceYToGoal, distanceXToGoal);

    var launchParams = config.shooter.launch_params;
    double launchPower;
    double waitTime;

    if (distanceToGoal <= launchParams.threshold_distance) {
      launchPower = launchParams.near.launch_power;
      waitTime = launchParams.near.wait_time;
    } else {
      double farBasePower =
          alliance == Alliance.RED
              ? launchParams.far.red_base_power
              : launchParams.far.blue_base_power;
      launchPower =
          farBasePower
              + ((distanceToGoal - launchParams.threshold_distance) / launchParams.far.power_scale);
      waitTime = distanceToGoal * launchParams.far.wait_time_scale;
    }

    return new LaunchParameters(launchPower, waitTime, Math.toDegrees(launchAngleRadians));
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
