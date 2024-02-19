package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class DropPurpleLeft1 {

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

    public DropPurpleLeft1(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }


    public SampleMecanumDrive drive = null;
    public Pose2d endPose;

    public Pose2d doPath(Pose2d startPose, Pose2d spikeDropPose) {

        drive = new SampleMecanumDrive(ahwMap);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(spikeDropPose, // line to left spikemark new Pose2d(-34, 30, Math.toRadians(0))
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(step1);

        endPose = step1.end();

        return endPose; // start pose for next step

    }

}
