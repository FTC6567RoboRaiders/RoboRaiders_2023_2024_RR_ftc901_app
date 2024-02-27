package RoboRaiders.Auto.RRTrajectorySteps;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.GlobalVariables;
import RoboRaiders.Robots.PirsusMkII;

public class giacomoStrafeToCommonPosition {



    HardwareMap ahwMap;

    public giacomoStrafeToCommonPosition(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }


    public SampleMecanumDrive drive = null;



    public void doPath(Pose2d startPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory redStage = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(convergePose, // line to converging position
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .strafeLeft(72)
                .build();
        Trajectory blueStage = drive.trajectoryBuilder(startPose)
                .strafeLeft(24)
                .build();

        if(GlobalVariables.getAllianceColour()){
            drive.followTrajectory(redStage);
        }
        else{
            drive.followTrajectory(blueStage);
        }



    }



}
