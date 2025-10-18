package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        final double pi = Math.PI;
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 14.50988120951799)
                .setDimensions(18, 16)
                .build();

        Pose2d beginPos = new Pose2d(11, -62, Math.toRadians(-90)); //new Pose2d(-10, -62, Math.toRadians(-90));

        DriveShim drive = myBot.getDrive();
        myBot.runAction(
                new SequentialAction(
                        drive.actionBuilder(beginPos)
                                .splineToConstantHeading(new Vector2d(11,-32.5), -pi/2, new TranslationalVelConstraint(30))
                                .build(),
                drive.actionBuilder(new Pose2d(11, -32.5, Math.toRadians(-90)))
                        //Drive to First Block
                        .splineToLinearHeading(new Pose2d(32, -15, pi/2), pi/2)
                        .splineToConstantHeading(new Vector2d(42, -12), -pi/2)
//                                // Push Block 1
                        .splineToConstantHeading(new Vector2d(45, -50), pi/2) // 54.5
//                                // Go back, Spline Turn, and go to block 2
//                                        .lineToY(-12)
                        .strafeTo(new Vector2d(40, -12))
                        .strafeTo(new Vector2d(52, -12))
//                                // Push Block 2
                        .splineToConstantHeading(new Vector2d(50, -50), pi/2) // 54.5
//                                // Go back, and go pick up Block 1
                        .splineToConstantHeading(new Vector2d(35, -52), -pi/2)
                        .splineToConstantHeading(new Vector2d(35, -60), -pi/2, new TranslationalVelConstraint(30))
                        .build()
                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}