package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;

@Configurable
@TeleOp
public class atan extends OpMode {

    private Follower follower;

    public static double GOAL_X = 129.0;
    public static double GOAL_Y = 132.0;

    public static double X = 45;
    public static double Y = 45;

    private FieldManager field;


    private final Pose initialPose = new Pose(72, 72, 0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(initialPose);
        follower.update();

                    field = PanelsField.INSTANCE.getField();
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
        Pose currentPose = follower.getPose();

        if (gamepad1.aWasPressed() && !follower.isBusy()) {

            double deltaX = GOAL_X - currentPose.getX();
            double deltaY = GOAL_Y - currentPose.getY();
            double targetHeading = Math.atan2(deltaY, deltaX);

            follower.turnTo(targetHeading);
        }
    }
}