package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import RoboRaiders.Robots.PirsusMkII;

public class SpikeToLoopBridge {

    HardwareMap ahwMap;

    public SpikeToLoopBridge(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public Pose2d endPose;
    public SampleMecanumDrive drive = null;



//    Pose2d startPose2 = new Pose2d(-35, 11.5, Math.toRadians(0));

    public Pose2d doPath(Pose2d startPose) {

        drive = new SampleMecanumDrive(ahwMap);

        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
//                .back(58, // drive to backdrop
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0), // spline up to backdrop
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .strafeRight(90)
                .build();

        drive.followTrajectory(step1);

        endPose = step1.end();
        return endPose;

    }

}
