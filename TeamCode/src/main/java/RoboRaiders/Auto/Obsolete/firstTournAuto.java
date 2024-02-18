package RoboRaiders.Auto.Obsolete;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import RoboRaiders.Auto.AutoOptions.AutoOptions;
import RoboRaiders.Pipelines.GripPipelineBlue;
import RoboRaiders.Pipelines.GripPipelineRed;
import RoboRaiders.Robots.Pirsus;


@Autonomous
@Disabled


public class firstTournAuto extends LinearOpMode {

    public Pirsus robot = new Pirsus();
    AutoOptions AO = new AutoOptions(this);

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    GripPipelineRed redPipeline;
    GripPipelineBlue bluePipeline;

    private boolean isRed = false;
    private boolean droneSide = false;
    private boolean driveInner = false;
    private boolean waitForPartner = false;

    private boolean selectionsAreGood = false;

    double numOfTicks;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.resetEncoders();
        robot.runWithEncoders();

        while (!selectionsAreGood) {

            isRed             = AO.selectAlliance();              // red or blue
            droneSide         = AO.selectStartLocation();         // starting near the drones or the backboard
 //           driveInner        = AO.selectDrivePath();      // drive near the centre or the wall
            waitForPartner    = AO.selectWait();                   // wait for partner

            // Add new/additional auto options, so things like drive to depot, drop team marker, etc..


            // Display the options selected
            // We show two options per line, to save space and lines.  The maximum number of characters
            // per line is roughly 45.  Maximum number of lines to be displayed is 9.
            // Note: To keep the autonomous options displayed, the automagical clearing of the telemetry data will be
            //       turned off with the setAutoClear(false) prior to calling selectionsGood().  After selectionsGood()
            //       turn on the automagical clearing of the telemetry data which is the default action.

            telemetry.setAutoClear(false);
            telemetry.addLine().addData("Autonomous", "Selections");
            telemetry.addLine().addData("Alliance:", isRed ? "Red  " : "Blue  ").addData("  Robot Start Location:", droneSide ? "Drone" : "Stage");
            telemetry.addLine().addData("Drive Side: ", driveInner ? "Inner" : "Wall");
            telemetry.addLine().addData("Wait for Partner:", waitForPartner ? "Yes" : "No");
            telemetry.update();

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, well ask again.

            selectionsAreGood = AO.selectionsGood();
            telemetry.setAutoClear(true);
            telemetry.update();    // Clear the selections
        }

        //    robot.initialize(hardwareMap);

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

        // red autos
        if(isRed) {

            // move off wall
            numOfTicks = robot.driveTrainCalculateCounts(2.0);
            robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
            while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
            }
            robot.resetEncoders();
            robot.runWithEncoders();
            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

            if(droneSide) {

                // move to drone wall
                numOfTicks = robot.driveTrainCalculateCounts(27.0);
                robot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // wait, if wait selected
                if(waitForPartner) {
                    sleep(3000);
                }

                // move off the wall
                numOfTicks = robot.driveTrainCalculateCounts(1.0);
                robot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // move to centre position
                numOfTicks = robot.driveTrainCalculateCounts(25.0);
                robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // pixel moving stuff goes here, adjust later motor powers as necessary
//                redPipeline.process(hardwareMap.get("Webcam 1"));



                // move to driveInner position, if selected
                if(driveInner) {

                    // move to inner drive position
                    numOfTicks = robot.driveTrainCalculateCounts(25.0);
                    robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move to outer position, if selected
                else {

                    // move to outer drive position
                    numOfTicks = robot.driveTrainCalculateCounts(25.0);
                    robot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move into drone wall
                numOfTicks = robot.driveTrainCalculateCounts(3.0);
                robot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // move to stage
                numOfTicks = robot.driveTrainCalculateCounts(125.0);
                robot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

            }

            else {

                // move closer to stage
                numOfTicks = robot.driveTrainCalculateCounts(25.0);
                robot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // move to centre position
                numOfTicks = robot.driveTrainCalculateCounts(25.0);
                robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // pixel moving stuff goes here, adjust later motor powers as necessary

                // still need to decide how the wait should be executed on this side

                // move to driveInner position, if selected
                if(driveInner) {

                    // move to inner drive position
                    numOfTicks = robot.driveTrainCalculateCounts(25.0);
                    robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move to driveInner position, if selected
                else {

                    // move to outer drive position
                    numOfTicks = robot.driveTrainCalculateCounts(27.0);
                    robot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move to stage
                numOfTicks = robot.driveTrainCalculateCounts(27.0);
                robot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

            }

        }

        // blue autos
        else {

            // move off wall
            numOfTicks = robot.driveTrainCalculateCounts(2.0);
            robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
            while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
            }
            robot.resetEncoders();
            robot.runWithEncoders();
            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

            if(droneSide) {

                // move to drone wall
                numOfTicks = robot.driveTrainCalculateCounts(27.0);
                robot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // wait, if wait selected
                if(waitForPartner) {
                    sleep(3000);
                }

                // move off the wall
                numOfTicks = robot.driveTrainCalculateCounts(1.0);
                robot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // move to centre position
                numOfTicks = robot.driveTrainCalculateCounts(25.0);
                robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // pixel moving stuff goes here, adjust later motor powers as necessary

                // move to driveInner position, if selected
                if(driveInner) {

                    // move to inner drive position
                    numOfTicks = robot.driveTrainCalculateCounts(25.0);
                    robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move to outer position, if selected
                else {

                    // move to outer drive position
                    numOfTicks = robot.driveTrainCalculateCounts(25.0);
                    robot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move into drone wall
                numOfTicks = robot.driveTrainCalculateCounts(3.0);
                robot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // move to stage
                numOfTicks = robot.driveTrainCalculateCounts(125.0);
                robot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

            }

            else {

                // move closer to stage
                numOfTicks = robot.driveTrainCalculateCounts(25.0);
                robot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // move to centre position
                numOfTicks = robot.driveTrainCalculateCounts(25.0);
                robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                // pixel moving stuff goes here, adjust later motor powers as necessary

                // still need to decide how the wait should be executed on this side

                // move to driveInner position, if selected
                if(driveInner) {

                    // move to inner drive position
                    numOfTicks = robot.driveTrainCalculateCounts(25.0);
                    robot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move to driveInner position, if selected
                else {

                    // move to outer drive position
                    numOfTicks = robot.driveTrainCalculateCounts(27.0);
                    robot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
                    while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                        telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                    }
                    robot.resetEncoders();
                    robot.runWithEncoders();
                    robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

                }

                // move to stage
                numOfTicks = robot.driveTrainCalculateCounts(27.0);
                robot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
                while (opModeIsActive() && robot.getSortedEncoderCount() <= numOfTicks) {
                    telemetry.addData("getSortEncoderCount()", robot.getSortedEncoderCount());
                }
                robot.resetEncoders();
                robot.runWithEncoders();
                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);

            }
        }

    }

}
