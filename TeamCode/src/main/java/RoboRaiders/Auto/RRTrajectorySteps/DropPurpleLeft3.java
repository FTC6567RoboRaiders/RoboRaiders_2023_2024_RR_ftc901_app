package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.PirsusMkII;

public class DropPurpleLeft3 {

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

    public DropPurpleLeft3(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = null;



    public void doPath(Pose2d startPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(convergePose, // line to converging position
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .strafeRight(23)
                .build();

        drive.followTrajectory(step1);
        drive.turn(Math.toRadians(90));

    }



}
