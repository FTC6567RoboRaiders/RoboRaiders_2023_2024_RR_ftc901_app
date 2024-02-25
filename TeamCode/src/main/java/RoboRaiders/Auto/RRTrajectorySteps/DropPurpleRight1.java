package RoboRaiders.Auto.RRTrajectorySteps;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.GlobalVariables;
import RoboRaiders.Robots.PirsusMkII;

public class DropPurpleRight1 {

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

    public DropPurpleRight1(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = null;
    public Pose2d endPose;
    public Pose2d intermediateEndPose = null;

    public Pose2d doPath(Pose2d startPose, Pose2d spikeDropPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .strafeRight(15)
                .build();
        Trajectory step3 = drive.trajectoryBuilder(startPose)
                .strafeLeft(7)
                .build();
        if((!GlobalVariables.getSide() && !GlobalVariables.getAllianceColour()) | (GlobalVariables.getSide() && GlobalVariables.getAllianceColour())) { // blue/backstage or red/stage
            intermediateEndPose = step1.end();
        }
        else if(!GlobalVariables.getSide() && GlobalVariables.getAllianceColour()) {
            intermediateEndPose = step3.end();
        }
        else {
            intermediateEndPose = startPose;
        }
            Trajectory step2 = drive.trajectoryBuilder(intermediateEndPose)
                .lineToLinearHeading(spikeDropPose, // line to right spikemark new Pose2d(-37, 30, Math.toRadians(180))
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        if((!GlobalVariables.getSide() && !GlobalVariables.getAllianceColour()) | (GlobalVariables.getSide() && GlobalVariables.getAllianceColour())) { // blue/backstage or red/stage
            drive.followTrajectory(step1);
        }
        else if(!GlobalVariables.getSide() && GlobalVariables.getAllianceColour()) { // red/backstage
            drive.followTrajectory(step3);
        }
        else {}
        drive.followTrajectory(step2);

        endPose = step1.end();

        return endPose; // returns endPose for next step

    }

}
