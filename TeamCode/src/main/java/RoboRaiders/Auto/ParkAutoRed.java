package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import RoboRaiders.Pipelines.GripPipelineBlue;
import RoboRaiders.Pipelines.GripPipelineRed;
import RoboRaiders.Robots.Pirsus;

@Autonomous
public class ParkAutoRed extends LinearOpMode {

    public Pirsus robot = new Pirsus();

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    GripPipelineRed redPipeline;
    GripPipelineBlue bluePipeline;

    double numOfTicks;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    @Override
    public void runOpMode() {

        robot.initialize(hardwareMap);

        robot.runWithEncoders();

        robot.resetEncoders();
        robot.runWithEncoders();

        gamepad1.reset();

//        if () {
//            robot.initVuforia();
//
//            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//            // first.
//
//            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//                robot.initTfod();
//            } else {
//                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//            }
//            /*if (robot.tfod != null) {
//                robot.tfod.activate();
//            }*/
//
//        }

        waitForStart();

        /** power controls:
         *  -, -, -, - | forward
         *  +, +, +, + | backward
         *  -, +, +, - | left
         *  +, -, -, + | right
         */

        // move off wall
        numOfTicks = robot.driveTrainCalculateCounts(0.10);
        robot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
        while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
            telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
        }
        robot.resetEncoders();
        robot.runWithEncoders();
        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

        // move backward
        numOfTicks = robot.driveTrainCalculateCounts(54.0);
        robot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
        while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
            telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
        }
        robot.resetEncoders();
        robot.runWithEncoders();
        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

    }

}