package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import RoboRaiders.Pipelines.StevesPipeline2;
import RoboRaiders.Robots.PirsusMkII;

@Autonomous
public class TournAuto extends LinearOpMode {

    public PirsusMkII robot = new PirsusMkII();

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag = null;
    public StevesPipeline2 stevesPipeline;

    @Override
    public void runOpMode() {

        robot.initialize(hardwareMap);

        robot.runWithEncoders();

        robot.resetEncoders();
        robot.runWithEncoders();

        gamepad1.reset();

        initAprilTag();

        waitForStart();


    }



}
