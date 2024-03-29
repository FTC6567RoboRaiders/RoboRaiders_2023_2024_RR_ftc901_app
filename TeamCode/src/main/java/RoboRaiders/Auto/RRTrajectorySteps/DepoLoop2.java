package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.PirsusMkII;

public class DepoLoop2 {

    //    order:
//    DPL1/2 or DPC or DPR1/2
//            |
//            |
//          STLB
//            |
//            |
//           DL1
//            |
//            |
//           DL2 < we are here
//
//    DL1 and DL2 cycle as necessary

    HardwareMap ahwMap;

    public DepoLoop2(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = null;



    Pose2d startPose = new Pose2d(-60, 11.5, Math.toRadians(0));

    public void doPath() {

        drive = new SampleMecanumDrive(ahwMap);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .back(82,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // move to stage
                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // spline to stage
                .build();

        drive.followTrajectory(step1);

    }

}