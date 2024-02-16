package RoboRaiders.Auto.Obsolete;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleCentre;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleLeft1;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleLeft2;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleRight1;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleRight2;
import RoboRaiders.Auto.RRTrajectorySteps.SpikeToLoopBridge;
import RoboRaiders.Robots.PirsusMkII;

@Autonomous
public class TournAuto extends LinearOpMode {

    public PirsusMkII robot = new PirsusMkII();
    public SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag = null;
//    public StevesPipeline2 stevesPipeline = new StevesPipeline2();
    public int position;
    public long startTime;
    public long elapsedTime;

    // RR path segments
    public DropPurpleLeft1 DPL1 = new DropPurpleLeft1(hardwareMap);
    public DropPurpleLeft2 DPL2 = new DropPurpleLeft2(hardwareMap);
    public DropPurpleCentre DPC = new DropPurpleCentre(hardwareMap);
    public DropPurpleRight1 DPR1 = new DropPurpleRight1(hardwareMap);
    public DropPurpleRight2 DPR2 = new DropPurpleRight2(hardwareMap);
    public DepoLoop1 depoLoop1 = new DepoLoop1(hardwareMap);
    public DepoLoop2 depoLoop2 = new DepoLoop2(hardwareMap);
    public SpikeToLoopBridge bridge = new SpikeToLoopBridge(hardwareMap);



    private static final boolean USE_WEBCAM = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        robot.runWithEncoders();

        robot.resetEncoders();
        robot.runWithEncoders();

        gamepad1.reset();

        initColorPortal();

//        position = propPosition();

        initAprilTagPortal();
        initColorPortal();

        Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);



        waitForStart();

        elapsedTime = System.nanoTime() - startTime;

        while(opModeIsActive()) {

            telemetryAprilTag();

            telemetry.update();

            // spike mark positions
            switch (position) {

                case 0:
                    DPL1.doPath(); // move to left spikemark
                    // drop pixel
                    DPL2.doPath(DPL1.doPath());
                    break;
                case 1:
                    DPC.doPath(); // move to centre spikemark
                    // drop pixel
                    break;
                case 2:
                    DPR1.doPath(); // move to right spikemark
                    // drop pixel
                    DPR2.doPath(DPL1.doPath());
                    break;
                default:
                    DPC.doPath(); // move to centre spikemark
                    break;

            }

//            bridge.doPath();

//            // intake/deposit block, repeat as necessary
//            depoLoop1.doPath();
//            // intake
//            depoLoop2.doPath();
//            //deposit

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
