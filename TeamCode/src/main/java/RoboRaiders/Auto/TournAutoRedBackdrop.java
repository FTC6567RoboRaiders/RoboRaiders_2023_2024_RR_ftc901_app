package RoboRaiders.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import RoboRaiders.Auto.RRTrajectorySteps.DepoLoop1;
import RoboRaiders.Auto.RRTrajectorySteps.DepoLoop2;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleCentre1;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleLeft1;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleLeft3;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleRight1;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleRight3;
import RoboRaiders.Auto.RRTrajectorySteps.SpikeToLoopBridge;
import RoboRaiders.Robots.Pirsus2;

@Autonomous
@Disabled
public class TournAutoRedBackdrop extends LinearOpMode {

    public Pirsus2 robot = new Pirsus2();
    public SampleMecanumDrive drive = null;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag = null;
    //    public StevesPipeline2 stevesPipeline = new StevesPipeline2();
    public int position;
    public long startTime;
    public long elapsedTime;
    public long depositTime;
    public long startTime1;
    public long depositTime1;

    // RR path segments
    public DropPurpleLeft1 DPL1 = null;
    public DropPurpleLeft3 DPL2 = null;
    public DropPurpleCentre1 DPC = null;
    public DropPurpleRight1 DPR1 = null;
    public DropPurpleRight3 DPR2 = null;
    public DepoLoop1 depoLoop1 = null;
    public DepoLoop2 depoLoop2 = null;
    public SpikeToLoopBridge bridge = null;
    public Pose2d endPose;
    public boolean pathCompleted = false;



    private static final boolean USE_WEBCAM = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        DPL1 = new DropPurpleLeft1(hardwareMap);
        DPL2 = new DropPurpleLeft3(hardwareMap);
        DPC = new DropPurpleCentre1(hardwareMap);
        DPR1 = new DropPurpleRight1(hardwareMap);
        DPR2 = new DropPurpleRight3(hardwareMap);
        depoLoop1 = new DepoLoop1(hardwareMap);
        depoLoop2 = new DepoLoop2(hardwareMap);
        bridge = new SpikeToLoopBridge(hardwareMap);



        robot.runWithEncoders();

        robot.resetEncoders();
        robot.runWithEncoders();

        gamepad1.reset();

//        initColorPortal();

//        position = propPosition();

//        initAprilTagPortal();
//        initColorPortal();





        waitForStart();

        elapsedTime = System.nanoTime() - startTime;

        position = 1;



        while(opModeIsActive() && !pathCompleted) {

//            telemetryAprilTag();

            telemetry.update();

            // spike mark positions
//            switch (position) {
//
//                case 0:
//                    DPL1.doPath(); // move to left spikemark
//                    // drop pixel
//                    DPL2.doPath(DPL1.doPath()); // move to centre location
//                    // drop pixel
//                    break;
//                case 1:
//                    endPose = DPC.doPath(); // move to centre spikemark
//                    // drop pixel
//                    break;
//                case 2:
//                    DPR1.doPath(); // move to right spikemark
//                    // drop pixel
//                    DPR2.doPath(DPL1.doPath());// move to centre location
//                    break;
//                default:
//                    DPC.doPath(); // move to centre spikemark
//                    break;
//
//            }


            Pose2d startPose = new Pose2d(10,-60, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            Trajectory step1 = drive.trajectoryBuilder(startPose)
//                .back(58, // drive to backdrop
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(0), // spline up to backdrop
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .back(5)
                    .build();



                drive.followTrajectory(step1);
                Pose2d endPose = step1.end();


            Trajectory step2 = drive.trajectoryBuilder(endPose)
                .strafeLeft(35)
                .build();

            drive.followTrajectory(step2);

            pathCompleted = true;



//            // deposit
//            startTime1 = System.nanoTime();
//            depositTime1 = System.nanoTime() - startTime;
//
//            while((depositTime1 / 1000000000) <= 3) {
//                robot.setIntakeMotorPower(1.0);
//                depositTime1 = System.nanoTime() - startTime1;
//            }
//
//            drive.setPoseEstimate(endPose);
//
//            Trajectory step3 = drive.trajectoryBuilder(endPose)
//                    .back(2)
//                    .build();
//
//            drive.followTrajectory(step3);
//            endPose = step1.end();
//
//            startTime = System.nanoTime();
//            depositTime = System.nanoTime() - startTime;
//
//            while((depositTime / 1000000000) <= 3) {
//                robot.setIntakeMotorPower(1.0);
//                depositTime = System.nanoTime()-startTime;
//            }
//
//            drive.setPoseEstimate(endPose);
//            drive.followTrajectory(step3);
//            endPose = step1.end();


            // intake/deposit block 1
//            depoLoop1.doPath();
//            // intake
//            depoLoop2.doPath();
//            //deposit
//
//            // intake/deposit block 2
//            depoLoop1.doPath();
//            // intake
//            depoLoop2.doPath();
            //deposit

        }



    }



    public void initAprilTagPortal() {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        }
        else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    public void initColorPortal() {

        // how?

    }

//    public int propPosition() {
//
//        if(stevesPipeline.returnX() <= 319) {
//            return 1;
//        }
//        else if(stevesPipeline.returnX() >= 320) {
//            return 2;
//        }
//        else {
//            return 0;
//        }
//
//    }

}
