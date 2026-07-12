package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.ConfigLoader;

@Configurable
public class Shooter {
    public static double MAX_RPM = ConfigLoader.getDouble("shooter.max_rpm");
    public static double minTransferThreashhold = ConfigLoader.getDouble("shooter.min_transfer_threshold");
    public static boolean AUTO_SHOOT_MODE = true;

    public static double PID_P = ConfigLoader.getDouble("shooter.pidf.p");
    public static double PID_I = ConfigLoader.getDouble("shooter.pidf.i");
    public static double PID_D = ConfigLoader.getDouble("shooter.pidf.d");
    public static double PID_F = ConfigLoader.getDouble("shooter.pidf.f");

    public static double CACHING_MOTOR_TOLERANCE = ConfigLoader.getDouble("caching.flywheel_tolerance");
    public static double CACHING_HOOD_TOLERANCE = ConfigLoader.getDouble("caching.hood_tolerance");

    public static double LAUNCH_THRESHOLD = ConfigLoader.getDouble("shooter.launch_params.threshold_distance");
    public static double NEAR_POWER = ConfigLoader.getDouble("shooter.launch_params.near.launch_power");
    public static double NEAR_WAIT = ConfigLoader.getDouble("shooter.launch_params.near.wait_time");
    
    public static double FAR_BASE_POWER = ConfigLoader.getDouble("shooter.launch_params.far.base_power");
    public static double RED_FAR_BASE_POWER = ConfigLoader.getDouble("shooter.launch_params.far.red_base_power");
    public static double FAR_POWER_SCALE = ConfigLoader.getDouble("shooter.launch_params.far.power_scale");
    public static double FAR_WAIT_SCALE = ConfigLoader.getDouble("shooter.launch_params.far.wait_time_scale");

    private final CachingDcMotorEx shooter1;
    private final CachingDcMotorEx shooter2;
    private final CachingServo hood;

    private boolean isShooting = false;
    private final ElapsedTime shootingTimer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        shooter1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter"));
        shooter2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter2"));
        hood = new CachingServo(hardwareMap.get(Servo.class, "hood"));

        shooter1.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        shooter2.setCachingTolerance(CACHING_MOTOR_TOLERANCE);
        hood.setCachingTolerance(CACHING_HOOD_TOLERANCE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        setShooterPIDFCoefficients();
        setHoodPosition(0);
    }

    public void setShooterPIDFCoefficients() {
        com.qualcomm.robotcore.hardware.PIDFCoefficients coef = new com.qualcomm.robotcore.hardware.PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coef);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coef);
    }

    public void runShooterRaw(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
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

    public void powerOnLauncher(double power) {
        double targetVel = MAX_RPM * power;
        shooter1.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);
    }

    public void startShooting() {
        isShooting = true;
        shootingTimer.reset();
    }

    public void stopShooting() {
        isShooting = false;
        stop();
    }

    public boolean isShooting() {
        return isShooting;
    }

    public double getShootingDuration() {
        return shootingTimer.milliseconds();
    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public void setHoodLongShotPosition() {
        hood.setPosition(0);
    }

    public LaunchParameters calculateLaunchParameters(Pose currentPose, double goalX, double goalY) {
        double deltaX = goalX - currentPose.getX();
        double deltaY = goalY - currentPose.getY();
        double distance = Math.hypot(deltaX, deltaY);
        double angleRadians = Math.atan2(deltaY, deltaX);

        double launchPower;
        double waitTime;

        if (distance <= LAUNCH_THRESHOLD) {
            launchPower = NEAR_POWER;
            waitTime = NEAR_WAIT;
        } else {
            launchPower = FAR_BASE_POWER + ((distance - LAUNCH_THRESHOLD) / FAR_POWER_SCALE);
            waitTime = distance * FAR_WAIT_SCALE;
        }

        return new LaunchParameters(launchPower, waitTime, Math.toDegrees(angleRadians));
    }

    public LaunchParameters REDcalculateLaunchParameters(Pose currentPose, double goalX, double goalY) {
        double deltaX = goalX - currentPose.getX();
        double deltaY = goalY - currentPose.getY();
        double distance = Math.hypot(deltaX, deltaY);
        double angleRadians = Math.atan2(deltaY, deltaX);

        double launchPower;
        double waitTime;

        if (distance <= LAUNCH_THRESHOLD) {
            launchPower = NEAR_POWER;
            waitTime = NEAR_WAIT;
        } else {
            launchPower = RED_FAR_BASE_POWER + ((distance - LAUNCH_THRESHOLD) / FAR_POWER_SCALE);
            waitTime = distance * FAR_WAIT_SCALE;
        }

        return new LaunchParameters(launchPower, waitTime, Math.toDegrees(angleRadians));
    }

    public Command shootCommand(double power) {
        return Command.build()
                .setStart(() -> powerOnLauncher(power))
                .setEnd(interrupted -> stop())
                .requiring(this);
    }

    public void takeShot(double launchPower, Intake intake) {
        if (!isShooting) return;
        double targetVel = MAX_RPM * launchPower;
        shooter1.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);
        if (shooter1.getVelocity() > (targetVel * minTransferThreashhold)) {
            intake.run(1.0, 1.0);
        } else {
            intake.stop();
        }
    }

    public void updateShootingLogic(double launchPower, Pose currentPose, Intake intake, Turret turret, Telemetry telemetry) {
        if (!isShooting) return;

        double targetVel = MAX_RPM * launchPower;
        shooter1.setVelocity(targetVel);
        shooter2.setVelocity(targetVel);

        telemetry.addData("current v", Math.abs(shooter1.getVelocity()));
        telemetry.addData("target v", Math.abs(targetVel) * minTransferThreashhold);

        double robotWorldHeading = currentPose.getHeading();
        double turretWorldAngle = turret.getCurrentTurnAngle();
        double targetWorldAngle = turret.getTargetTurnAngle();

        double relativeTurretAngle = (turretWorldAngle - robotWorldHeading + 360) % 360;
        double relativeTargetAngle = (targetWorldAngle - robotWorldHeading + 360) % 360;

        boolean turnLeft = turret.shouldTurnLeft(relativeTurretAngle, relativeTargetAngle);
        double error;
        if (turnLeft) {
            error = (relativeTargetAngle - relativeTurretAngle + 360) % 360;
            if (error > 180) error -= 360;
        } else {
            error = -((relativeTurretAngle - relativeTargetAngle + 360) % 360);
            if (error < -180) error += 360;
        }

        double distance = Math.hypot(turret.getGoalX() - currentPose.getX(), turret.getGoalY() - currentPose.getY());
        double dynamicTolerance = turret.calculateDynamicTolerance(distance);

        boolean velocityReady = Math.abs(shooter1.getVelocity()) > Math.abs(targetVel) * minTransferThreashhold &&
                                Math.abs(shooter1.getVelocity()) < Math.abs(targetVel) * 1.05;
        boolean turretAligned = Math.abs(error) < dynamicTolerance;

        telemetry.addData("Turret Error (deg)", error);
        telemetry.addData("Dynamic Tolerance (deg)", dynamicTolerance);
        telemetry.addData("Turret Aligned", turretAligned);

        if (velocityReady && turretAligned) {
            intake.run(1.0, 1.0);
        } else {
            intake.stop();
        }
    }
}
