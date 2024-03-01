package RoboRaiders.Auto.RRTrajectorySteps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robots.GlobalVariables;

public class SpikeToLoopBridgeNew {

    //    order:
//    DPL1/2 or DPC or DPR1/2
//            |
//            |
//          STLB < we are here
//            |
//            |
//           DL1
//            |
//            |
//           DL2
//
//    DL1 and DL2 cycle as necessary

    HardwareMap ahwMap;

    public SpikeToLoopBridgeNew(HardwareMap ahwMap) {

        this.ahwMap = ahwMap;

    }

    public Pose2d endPose;
    public Pose2d intermediateEndPose;
    public Pose2d firstIntEndPose;
    public SampleMecanumDrive drive = null;



//    Pose2d startPose2 = new Pose2d(-35, 11.5, Math.toRadians(0));

    public Pose2d doPath(Pose2d startPose, Vector2d lineToPose, Vector2d lineToPose2, Vector2d lineToPose3) {

        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(startPose);

        if(GlobalVariables.getAllianceColour() && GlobalVariables.getParkLeft()) {
            Trajectory stepR1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(lineToPose)
                    .build();
            drive.followTrajectory(stepR1);
            intermediateEndPose = stepR1.end();
            Trajectory stepR3 = drive.trajectoryBuilder(intermediateEndPose)
                    .lineToConstantHeading(lineToPose3)
                    .build();
            drive.followTrajectory(stepR3);
        }
        else if(GlobalVariables.getAllianceColour() && !GlobalVariables.getParkLeft()) {
            Trajectory stepR1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(lineToPose)
                    .build();
            drive.followTrajectory(stepR1);
            intermediateEndPose = stepR1.end();
            Trajectory stepR2 = drive.trajectoryBuilder(intermediateEndPose)
                    .forward(50)
                    .build();
            drive.followTrajectory(stepR2);
            intermediateEndPose = stepR2.end();
            Trajectory stepR3 = drive.trajectoryBuilder(intermediateEndPose)
                    .lineToConstantHeading(lineToPose3)
                    .build();
            drive.followTrajectory(stepR3);
        }
        else if(!GlobalVariables.getAllianceColour() && GlobalVariables.getParkLeft()) {
            Trajectory stepB1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(lineToPose)
                    .build();
            drive.followTrajectory(stepB1);
            intermediateEndPose = stepB1.end();
            Trajectory stepB2 = drive.trajectoryBuilder(intermediateEndPose)
                    .forward(50)
                    .build();
            drive.followTrajectory(stepB2);
            intermediateEndPose = stepB2.end();
            Trajectory stepB3 = drive.trajectoryBuilder(intermediateEndPose)
                    .lineToConstantHeading(lineToPose3)
                    .build();
            drive.followTrajectory(stepB3);
        }
        else {
            Trajectory stepB1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(lineToPose)
                    .build();
            drive.followTrajectory(stepB1);
            intermediateEndPose = stepB1.end();
            Trajectory stepB3 = drive.trajectoryBuilder(intermediateEndPose)
                    .lineToConstantHeading(lineToPose3)
                    .build();
            drive.followTrajectory(stepB3);
        }

        endPose = null;
        return endPose;

    }

}
