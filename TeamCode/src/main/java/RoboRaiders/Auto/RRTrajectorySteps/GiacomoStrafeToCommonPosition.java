package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.GlobalVariables;

public class GiacomoStrafeToCommonPosition {



    HardwareMap ahwMap;

    public GiacomoStrafeToCommonPosition(HardwareMap ahwMap) {

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
        Trajectory blueBackstage = drive.trajectoryBuilder(startPose)
                .strafeLeft(24)
                .build();

        if(GlobalVariables.getAllianceColour() && GlobalVariables.getSide()){ //If red & stage then strafe all the way over 72 inches
            drive.followTrajectory(redStage);
        }
        else{ //If blue & backstage strafe 24 inches
     //       drive.followTrajectory(blueBackstage);
        }



    }



}
