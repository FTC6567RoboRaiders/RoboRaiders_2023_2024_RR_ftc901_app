package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.GlobalVariables;

public class Park {



    HardwareMap ahwMap;

    public Park(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }


    public SampleMecanumDrive drive = null;



    public void doPath(Pose2d startPose) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        Trajectory forward = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(convergePose, // line to converging position
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(48)
                .build();

        Trajectory strafeRight = drive.trajectoryBuilder(startPose)
                .strafeRight(15)
                .build();
        Trajectory strafeLeft = drive.trajectoryBuilder(startPose)
                .strafeLeft(15)
                .build();


        if(GlobalVariables.getAllianceColour() && GlobalVariables.getParkLeft()){ //If red & park left then just strafe 15 inches left
            drive.followTrajectory(strafeLeft);
        }
        else if(GlobalVariables.getAllianceColour() && !GlobalVariables.getParkLeft()){ //If red and right park
            drive.followTrajectory(forward);
            drive.followTrajectory(strafeLeft);
        }
        else if(!GlobalVariables.getAllianceColour() && GlobalVariables.getParkLeft()){ //If blue & park left then drive forward and then strafe right
            drive.followTrajectory(forward);
            drive.followTrajectory(strafeRight);
        }
        else{
            drive.followTrajectory(strafeRight);
        }



    }



}
