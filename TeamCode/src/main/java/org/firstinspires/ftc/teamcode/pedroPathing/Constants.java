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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
  public static FollowerConstants followerConstants =
      new FollowerConstants()
          .mass(11.15)
          .headingPIDFCoefficients(new PIDFCoefficients(.7, 0, 0.002, 0.02))
          .predictiveBrakingCoefficients(
              new PredictiveBrakingCoefficients(0.05, 0.05872647384322376, 0.001561731123457261))
          .centripetalScaling(0);

  public static PathConstraints pathConstraints = new PathConstraints(0.90, 100, 1, 1);

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
          .xVelocity(75.64281986)
          .yVelocity(58.9247686);

  public static PinpointConstants localizerConstants =
      new PinpointConstants()
          .forwardPodY(1.5729952)
          .strafePodX(-4.535451)
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

  /** Builds the Follower with standard hardware map. */
  public static Follower createCachedFollower(HardwareMap hardwareMap) {
    return createFollower(hardwareMap);
  }
}
