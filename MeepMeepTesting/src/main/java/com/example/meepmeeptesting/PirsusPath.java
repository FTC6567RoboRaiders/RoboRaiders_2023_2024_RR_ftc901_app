package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PirsusPath {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        // startpos coords:
        // blue/drone side - (-35, 60)
        // blue/stage side - (12, 60)
        // red/drone side - (-35, -60)
        // red/stage side - (12, -60)

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13,17)    // Set the dimensions of the robot
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(90)))
                                        .back(25)
//   --------------------------------------------------------------------------------------------
//   place the purple pixel after forward
//   --------------------------------------------------------------------------------------------
                                        .waitSeconds(3)
                                        .splineTo(new Vector2d(3, 2), Math.toRadians(0))
//                                .splineTo(new Vector2d(-31, 34),Math.toRadians(180))
//                                .splineTo(new Vector2d(-24, 35),Math.toRadians(180))
                                        .back(75)
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