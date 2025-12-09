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

    public static double GOAL_X = 135.0;
    public static double GOAL_Y = 135.0;

    public static double X = 45;
    public static double Y = 45;

    public double prevx, prevy;

    private FieldManager field;


    private final Pose initialPose = new Pose(87.0, 8.0, Math.PI / 2.0);

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
        DrawingUtil.drawRobotOnField(field, X, Y, Math.atan2(GOAL_Y - Y, GOAL_X - X), GOAL_X, GOAL_Y);

        if (prevx != X || prevy != Y) {
            Pose currentPose = follower.getPose();

            double deltaX = GOAL_X - X;
            double deltaY = GOAL_Y - Y;
            double targetHeading = Math.atan2(deltaY, deltaX);


            Pose finalTarget = new Pose(X, Y, targetHeading);

            Path scorePreload = new Path(new BezierLine(follower.getPose(), finalTarget));
            scorePreload.setLinearHeadingInterpolation(follower.getHeading(), targetHeading);
            follower.followPath(scorePreload, true);
        }
        prevy = Y;
        prevx = X;
    }
}