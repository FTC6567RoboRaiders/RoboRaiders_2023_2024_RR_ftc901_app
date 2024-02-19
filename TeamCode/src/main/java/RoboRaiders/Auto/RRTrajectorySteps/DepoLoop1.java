package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.PirsusMkII;

public class DepoLoop1 {

//    order:
//    DPL1/2 or DPC or DPR1/2
//            |
//            |
//          STLB
//            |
//            |
//           DL1 < we are here
//            |
//            |
//           DL2
//
//    DL1 and DL2 cycle as necessary

    HardwareMap ahwMap;

    public DepoLoop1(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = null;



    Pose2d startPose = new Pose2d(47, 35, Math.toRadians(0));

    public void doPath() {

        drive = new SampleMecanumDrive(ahwMap);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(23, 11.5), Math.toRadians(180), // spline to pixel level
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(83,  // move to pixel
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(step1);

    }

}