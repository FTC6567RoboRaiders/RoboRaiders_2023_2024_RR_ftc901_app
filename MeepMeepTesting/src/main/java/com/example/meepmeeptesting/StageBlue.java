package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class StageBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13,17)    // Set the dimensions of the robot

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(270)))

                                .forward(30)
                                .turn(Math.toRadians(90))
                                .turn(Math.toRadians(-90))

                                .back(12)
                                .turn(Math.toRadians(-90))
                                .back(36)
                                .strafeLeft(28)
                                .forward(84)
                                .splineToConstantHeading(new Vector2d(-55.5, 23.5),Math.toRadians(0))
                                .forward(4)
                                .back(4)
                                .splineToConstantHeading(new Vector2d(-35, 13), Math.toRadians(0))
                                .back(84)
                                .strafeRight(20)
                                .strafeLeft(20)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}