package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.GlobalVariables;

public class DropPurpleCentre1 {

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

    public DropPurpleCentre1(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public Pose2d endPose;
    public SampleMecanumDrive drive = null;
    public Pose2d intermediateEndPose = null;

    public Pose2d doPath(Pose2d startPose, Vector2d lineToEndPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
//                .back(48, // drive to converging position
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(lineToEndPose)
                .build();
        intermediateEndPose = step1.end();
        Trajectory step2Left = drive.trajectoryBuilder(intermediateEndPose)
                .strafeLeft(12)
                .build();
        Trajectory step2Left2 = drive.trajectoryBuilder(intermediateEndPose)
                .strafeLeft(15)
                .build();
        Trajectory step2Right = drive.trajectoryBuilder(intermediateEndPose)
                .strafeRight(12)
                .build();

        drive.followTrajectory(step1);
        if(!GlobalVariables.getAllianceColour() && !GlobalVariables.getSide()) {
            drive.followTrajectory(step2Left);
        }
        else if(GlobalVariables.getAllianceColour() && GlobalVariables.getSide()) {
            drive.followTrajectory(step2Left2);
        }
        else if((GlobalVariables.getAllianceColour() && !GlobalVariables.getSide()) | (!GlobalVariables.getAllianceColour() && GlobalVariables.getSide())) {
            drive.followTrajectory(step2Right);
        }

        endPose = step1.end();
        return endPose; //Start pose for next step

    }

}
