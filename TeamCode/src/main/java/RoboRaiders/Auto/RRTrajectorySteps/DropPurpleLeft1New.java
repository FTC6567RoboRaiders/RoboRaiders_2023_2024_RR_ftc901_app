package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import RoboRaiders.Robots.GlobalVariables;

public class DropPurpleLeft1New {

    //    order:
//    DPL1/2 or DPC or DPR1/2 < we are here
//            |
//            |
//          STLB
//            |
//            |
//           DL1
//            |
//            |
//           DL2
//
//    DL1 and DL2 cycle as necessary

    HardwareMap ahwMap;

    public DropPurpleLeft1New(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }


    public SampleMecanumDrive drive = null;
    public Pose2d endPose;
    public Pose2d intermediateEndPose = null;

    public Pose2d doPath(Pose2d startPose, Pose2d spikeDropPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        // For Blue alliance and stage  OR Red Alliance and Backstage
        // strafe 15 inches to the left to align the robot better to deposit the pixel
        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(15)
                .build();



        //This is path we follow when backstage & blue alliance or stage & red alliance
        Trajectory giacomoStep1 = drive.trajectoryBuilder(startPose)
                .strafeRight(12)
                .build();


        // When stage and blue alliance -OR- backstage and red alliance, the intermediate pose is the end of step1 (the strafe left)
        if((GlobalVariables.getSide() && !GlobalVariables.getAllianceColour()) | (!GlobalVariables.getSide() && GlobalVariables.getAllianceColour())) { // blue/stage or red/backstage
            intermediateEndPose = step1.end();
        }
        // When backstage and blue alliance -OR- red alliance and stage, the intermediate pose is the end pose of the strafe right for optimized auto
        else {
            intermediateEndPose = giacomoStep1.end();
        }
        // Step2 align to the spike mark with a Pose2d(-34, 30, Math.toRadians(0))
        Trajectory step2 = drive.trajectoryBuilder(intermediateEndPose)
                .lineToLinearHeading(spikeDropPose, // line to left spikemark new Pose2d(-34, 30, Math.toRadians(0))
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory giacomoStep2 = drive.trajectoryBuilder(intermediateEndPose)
                .back(40)
                .build();



        // When stage and blue alliance -OR- backstage and red alliance, follow step 1 trajectory - strafe left 15 inches, position robot to deposit
        if((GlobalVariables.getSide() && !GlobalVariables.getAllianceColour()) | (!GlobalVariables.getSide() && GlobalVariables.getAllianceColour())) { // blue/stage or red/backstage
            drive.followTrajectory(step1);
            drive.followTrajectory(step2);
        }

        // When backstage and blue alliance -OR- red alliance and stage.
        // Strafe 12 inches right then drive back 30
        else {
            drive.followTrajectory(giacomoStep1);
            drive.followTrajectory(giacomoStep2);
        }

        // Now turn the robot so that we can deposit the purple pixel on the left spike (deposit is not done here)


        endPose = step1.end();   //???should this be step.end()

        return endPose; // start pose for next step

    }

}
