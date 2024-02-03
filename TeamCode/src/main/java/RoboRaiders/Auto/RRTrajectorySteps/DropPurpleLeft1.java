package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.PirsusMkII;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class DropPurpleLeft1 {

    HardwareMap ahwMap;

    public DropPurpleLeft1(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = new SampleMecanumDrive(ahwMap);
    public Pose2d endPose;



    Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(90));

    public Pose2d doPath() {

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34, 30, Math.toRadians(0)), // line to left spikemark
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(step1);

        endPose = step1.end();

        return endPose; // start pose for next step

    }

}
