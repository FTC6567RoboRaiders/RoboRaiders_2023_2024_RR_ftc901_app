package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.PirsusMkII;

public class DropPurpleRight2 {

    HardwareMap ahwMap;

    public DropPurpleRight2(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = new SampleMecanumDrive(ahwMap);
    public Pose2d endPose;



    public Pose2d doPath(Pose2d startPose) {

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-35, 11.5), // line to converging position
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(step1);

        endPose = step1.end();

        return endPose; // returns endPose for next step

    }

}
