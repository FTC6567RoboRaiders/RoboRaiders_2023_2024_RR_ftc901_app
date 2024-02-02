package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PirsusPath3 {
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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(90)))
                                .back(18) // move off wall
                                .back(1) // fixes spline going forward issue
                                .splineTo(new Vector2d(-37, 34), Math.toRadians(0)) // spline to door
                                .waitSeconds(2) // placeholder for dropping purple
                                .back(24) // drive to stage
                                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0)) // spline to stage
                                .waitSeconds(3) // placeholder for dropping yellow
                                // end initial cycle

                                // begin cycle 2
                                .lineToConstantHeading(new Vector2d(26, 11.5))
                                .forward(84) // move to pixel
                                .waitSeconds(1) // placeholder for collecting pixels
                                .back(82) // move to stage
                                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0)) // spline to stage
                                .waitSeconds(3) // placeholder for dropping pixels
                                // end cycle 2

                                // begin cycle 3
                                .lineToConstantHeading(new Vector2d(26, 11.5))
                                .forward(84) // move to pixel
                                .waitSeconds(1) // placeholder for collecting pixels
                                .back(82) // move to stage
                                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0)) // spline to stage
                                .waitSeconds(3) // placeholder for dropping pixels
                                // end cycle 3

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