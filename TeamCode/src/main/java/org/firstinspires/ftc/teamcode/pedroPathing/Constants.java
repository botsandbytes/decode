package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.6)
            .forwardZeroPowerAcceleration(33.93405724043018) //-32.18050856840037
            .lateralZeroPowerAcceleration(-55.73074354916248) //-73.31581854009389
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0.001, 0.03, 0)) // 0.5
//            .headingPIDFCoefficients(new PIDFCoefficients(.045, 0, 0, 0.02)) // d - 0,
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.003, 0.021)) // 0.5
            .headingPIDFCoefficients(new PIDFCoefficients(.7, 0, 0.002, 0.01)) // d - 0,
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.014, 0, 0.0005, 0.6, 0.0)) // amit p;0.017 and f - 0
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0025, 0, 0, 1.5, 0.02)) // amit p;0.017 and f - 0
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            .35,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.5641907218873) // 89.39393051027312
            .yVelocity(67.41060656449926); // 64.57908233882874

    public static PinpointConstants localizerConstants = new PinpointConstants()
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
}
