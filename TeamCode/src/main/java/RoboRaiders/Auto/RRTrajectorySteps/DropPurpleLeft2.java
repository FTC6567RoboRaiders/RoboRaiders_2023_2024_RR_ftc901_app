package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.GlobalVariables;

public class DropPurpleLeft2 {

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

    public DropPurpleLeft2(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public Pose2d endPose;
    public SampleMecanumDrive drive = null;

    public Pose2d doPath(Pose2d startPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .back(3, // drive to converging position
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory step2 = drive.trajectoryBuilder(startPose)
                .back(5, // drive to converging position
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //These Giacomo steps will be for the strafing code on blue backstage or red stage
        Trajectory giacomoStep1 = drive.trajectoryBuilder(startPose)
                .back(5)
                .build();

        Pose2d giacomoEndPose = giacomoStep1.end();

        Trajectory giacomoStrafeRightStep2 = drive.trajectoryBuilder(giacomoEndPose)
                .strafeRight(12)
                .build();

        Trajectory giacomoStrafeLeftStep2 = drive.trajectoryBuilder(giacomoEndPose)
                .strafeLeft(12)
                .build();





        // When stage and blue alliance -OR- backstage and red alliance, the intermediate pose is the end of step1 (the strafe left)
        if((GlobalVariables.getSide() && !GlobalVariables.getAllianceColour()) | (!GlobalVariables.getSide() && GlobalVariables.getAllianceColour())) { // blue/stage or red/backstage            drive.followTrajectory(step2);
            drive.followTrajectory(step1);
            drive.followTrajectory(step2);
        }
        // When red alliance and stage, the intermediate pose is the end pose of the strafe left for optimized auto
        else if (GlobalVariables.getAllianceColour() && GlobalVariables.getSide() ){
            drive.followTrajectory(giacomoStep1);
            drive.followTrajectory(giacomoStrafeLeftStep2);

        }
        // When blue alliance and backstage strafe right
        else if (!GlobalVariables.getAllianceColour() && !GlobalVariables.getSide() ){
            drive.followTrajectory(giacomoStep1);
            drive.followTrajectory(giacomoStrafeRightStep2);
        }

        endPose = step1.end();
        return endPose; //Start pose for next step

    }

}
