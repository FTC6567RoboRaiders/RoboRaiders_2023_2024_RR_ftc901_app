package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DroneBlueSteeeveLeftStageDoor {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,12)    // Set the dimensions of the robot
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(270)))
                                .forward(25)
                                .turn(Math.toRadians(-90))
//   --------------------------------------------------------------------------------------------
//   place the purple pixel at this point
//   --------------------------------------------------------------------------------------------
                                .turn(Math.toRadians(90))
                                .forward(22)
                                .turn(Math.toRadians(-90))
                                .back(84)
                                .strafeRight(28)
// deposit yellow pixel
                                .strafeLeft(28)
                                .forward(84)
                                .splineToConstantHeading(new Vector2d(-55, 23.5),Math.toRadians(0))
                                .forward(5)
// grab two more pixels
                                .back(5)
                                .splineToConstantHeading(new Vector2d(-35, 13), Math.toRadians(0))
                                .back(84)
                                .strafeRight(20)
                                .strafeLeft(20)
// deposit two pixels
//                                .splineTo(new Vector2d(-24, 35),Math.toRadians(180))
//                                .back(60)
//   --------------------------------------------------------------------------------------------
//   now in position to put the yellow pixel on the backdrop
//   --------------------------------------------------------------------------------------------
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}