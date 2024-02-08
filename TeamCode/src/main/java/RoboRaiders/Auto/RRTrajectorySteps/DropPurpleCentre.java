package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.PirsusMkII;

public class DropPurpleCentre {

    HardwareMap ahwMap;

    public DropPurpleCentre(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public Pose2d endPose;
    public SampleMecanumDrive drive = null;



    Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(90));

    public Pose2d doPath() {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .back(50, // drive to converging position
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(step1);

        endPose = step1.end();
        return endPose; //Start pose for next step

    }

}
