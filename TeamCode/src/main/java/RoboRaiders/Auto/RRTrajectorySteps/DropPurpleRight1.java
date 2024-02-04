package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.PirsusMkII;

public class DropPurpleRight1 {

    HardwareMap ahwMap;

    public DropPurpleRight1(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = null;
    public Pose2d endPose;



    Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(90));

    public Pose2d doPath() {

        drive = new SampleMecanumDrive(ahwMap);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-37, 30, Math.toRadians(180)), // line to right spikemark
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(step1);

        endPose = step1.end();

        return endPose; // returns endPose for next step

    }

}
