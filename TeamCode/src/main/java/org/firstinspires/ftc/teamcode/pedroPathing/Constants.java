package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.config.config;

public class Constants {
  public static FollowerConstants followerConstants =
      new FollowerConstants()
          .mass(8.6)
          .headingPIDFCoefficients(new PIDFCoefficients(.7, 0, 0.002, 0.01))
          .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016))
          .centripetalScaling(0);

  public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .35, 1);

  public static MecanumConstants driveConstants =
      new MecanumConstants()
          .maxPower(1)
          .rightFrontMotorName("rightFront")
          .rightRearMotorName("rightBack")
          .leftRearMotorName("leftBack")
          .leftFrontMotorName("leftFront")
          .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
          .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
          .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
          .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
          .xVelocity(80.5641907218873) // 89.39393051027312
          .yVelocity(67.41060656449926); // 64.57908233882874

  public static PinpointConstants localizerConstants =
      new PinpointConstants()
          .forwardPodY(1.5)
          .strafePodX(-4.75)
          // center of robot -6.5
          .distanceUnit(DistanceUnit.INCH)
          .hardwareMapName("pinpoint")
          .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
          .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
          .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pinpointLocalizer(localizerConstants)
        .pathConstraints(pathConstraints)
        .mecanumDrivetrain(driveConstants)
        .build();
  }

  /**
   * Wraps all four drivetrain motors in {@link CachingDcMotorEx}, re-registers them into {@code
   * hardwareMap} so Pedro Pathing picks up the cached instances, and builds the Follower.
   */
  public static Follower createCachedFollower(HardwareMap hardwareMap) {
    double drivetrainTolerance = config.caching.drivetrain_tolerance;
    CachingDcMotorEx lf = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
    CachingDcMotorEx lb = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftBack"));
    CachingDcMotorEx rf = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
    CachingDcMotorEx rb = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBack"));

    lf.setCachingTolerance(drivetrainTolerance);
    lb.setCachingTolerance(drivetrainTolerance);
    rf.setCachingTolerance(drivetrainTolerance);
    rb.setCachingTolerance(drivetrainTolerance);

    hardwareMap.put("leftFront", lf);
    hardwareMap.put("leftBack", lb);
    hardwareMap.put("rightFront", rf);
    hardwareMap.put("rightBack", rb);

    return createFollower(hardwareMap);
  }
}
