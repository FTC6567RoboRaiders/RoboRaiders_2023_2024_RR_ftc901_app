package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.graalvm.compiler.lir.phases.PostAllocationOptimizationStage_OptionDescriptors;

public class PirsusPathBlueStage {
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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                                .back(40) // move off wall
                                .waitSeconds(2) // placeholder for dropping purple
//                                .splineTo(new Vector2d(0, 11.5), Math.toRadians(0)) // spline to door
//                                .back(24) // drive to stage
                                .lineToLinearHeading(new Pose2d(47, 35, Math.toRadians(180)))
                                .waitSeconds(2) // placeholder for dropping yellow
                                // end initial cycle

                                // begin cycle 2
//                                .lineToConstantHeading(new Vector2d(26, 11.5))
                                .splineTo(new Vector2d(23, 11.5), Math.toRadians(180))
                                .forward(84) // move to pixel
                                .waitSeconds(1) // placeholder for collecting pixels
                                .back(82) // move to stage
                                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0)) // spline to stage
                                .waitSeconds(2) // placeholder for dropping pixels
                                // end cycle 2

                                // begin cycle 3
                                .splineTo(new Vector2d(23, 11.5), Math.toRadians(180))
                                .forward(84) // move to pixel
                                .waitSeconds(1) // placeholder for collecting pixels
                                .back(82) // move to stage
                                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0)) // spline to stage
                                .waitSeconds(2) // placeholder for dropping pixels
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