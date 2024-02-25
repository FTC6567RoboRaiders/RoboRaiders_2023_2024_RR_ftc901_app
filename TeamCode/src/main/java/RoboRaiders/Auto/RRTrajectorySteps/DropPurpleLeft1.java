package RoboRaiders.Auto.RRTrajectorySteps;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import RoboRaiders.Robots.GlobalVariables;

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
    public Pose2d intermediateEndPose = null;

    public Pose2d doPath(Pose2d startPose, Pose2d spikeDropPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(15)
                .build();
        if((GlobalVariables.getSide() && !GlobalVariables.getAllianceColour()) | (!GlobalVariables.getSide() && GlobalVariables.getAllianceColour())) { // blue/stage or red/backstage
            intermediateEndPose = step1.end();
        }
        else {
            intermediateEndPose = startPose;
        }
        Trajectory step2 = drive.trajectoryBuilder(intermediateEndPose)
                .lineToLinearHeading(spikeDropPose, // line to left spikemark new Pose2d(-34, 30, Math.toRadians(0))
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        if((GlobalVariables.getSide() && !GlobalVariables.getAllianceColour()) | (!GlobalVariables.getSide() && GlobalVariables.getAllianceColour())) { // blue/stage or red/backstage
            drive.followTrajectory(step1);
        }
        else {}
        drive.followTrajectory(step2);

        endPose = step1.end();

        return endPose; // start pose for next step

    }

}
