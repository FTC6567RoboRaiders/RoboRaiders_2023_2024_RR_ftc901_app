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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import RoboRaiders.Auto.AutoOptions.AutoOptions;
import RoboRaiders.Auto.RRTrajectorySteps.DepoLoop1;
import RoboRaiders.Auto.RRTrajectorySteps.DepoLoop2;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleCentre;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleLeft1;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleLeft2;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleRight1;
import RoboRaiders.Auto.RRTrajectorySteps.DropPurpleRight2;
import RoboRaiders.Auto.RRTrajectorySteps.SpikeToLoopBridge;
import RoboRaiders.Robots.GlobalVariables;
import RoboRaiders.Robots.Pirsus2;

@Autonomous
@Disabled

public class PirsusAuto extends LinearOpMode {

    public Pirsus2 robot = new Pirsus2();
    public SampleMecanumDrive drive = null;
    AutoOptions AO = new AutoOptions(this);

    OpenCvCamera camera;
    public WebcamName webcam1;

    int cameraMonitorViewId;

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
    public DropPurpleLeft2 DPL2 = null;
    public DropPurpleCentre DPC = null;
    public DropPurpleRight1 DPR1 = null;
    public DropPurpleRight2 DPR2 = null;
    public DepoLoop1 depoLoop1 = null;
    public DepoLoop2 depoLoop2 = null;
    public SpikeToLoopBridge bridge = null;
    public Pose2d endPose;
    public boolean pathCompleted = false;

    // AutoOptions stuff
    public boolean isRed = false;
    public Pose2d initialPos;
    public boolean droneSide = false;
    public boolean waitForPartner = false;
    public boolean selectionsAreGood = false;
    public Pose2d DPL2StartPos;
    public Pose2d DPR2StartPos;
    public Pose2d bridgeStartPos;
    public Pose2d DL1StartPos;
    public Pose2d DL2StartPos;





    private static final boolean USE_WEBCAM = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        DPL1 = new DropPurpleLeft1(hardwareMap);
        DPL2 = new DropPurpleLeft2(hardwareMap);
        DPC = new DropPurpleCentre(hardwareMap);
        DPR1 = new DropPurpleRight1(hardwareMap);
        DPR2 = new DropPurpleRight2(hardwareMap);
        depoLoop1 = new DepoLoop1(hardwareMap);
        depoLoop2 = new DepoLoop2(hardwareMap);
        bridge = new SpikeToLoopBridge(hardwareMap);

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(robot.stevesPipeline);

        camera.openCameraDeviceAsync(new  OpenCvCamera.AsyncCameraOpenListener()


        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);


//                Logger Log = new Logger(String.valueOf("******** ON OPENED *******"));
//                Log.Debug("X COORDINATE: ", robot.getX());
//                Log.Debug("Y COORDINATE: ", robot.getY());
//                Log.Debug("X 'Bottom COORDINATE: ", robot.getBX());
//                Log.Debug("Y Bottom COORDINATE: ", robot.getBY());


            }

            @Override
            public void onError(int errorCode)
            {
                // For now do nothing when we have an error
            }

        });

        while (!selectionsAreGood) {

            isRed = AO.selectAlliance();              // red or blue
            droneSide = AO.selectStartLocation();         // starting near the drones or the backboard
            waitForPartner = AO.selectWait();                   // wait for partner

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
            telemetry.addLine().addData("Wait for Partner:", waitForPartner ? "Yes" : "No");
            telemetry.update();

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, we'll ask again.

            selectionsAreGood = AO.selectionsGood();
            telemetry.setAutoClear(true);
            telemetry.update();    // Clear the selections
        }

        // coords for start positions:
        // red/drone: (-35, -60)
        // red/stage: (10, -60)
        // blue/drone: (-35, 60)
        // blue/stage: (10, 60)

        if(isRed && droneSide) { // red/drone
            initialPos = new Pose2d(-35, -60, Math.toRadians(270));
        }
        if(isRed && !droneSide) { // red/stage
            initialPos = new Pose2d(10, -60, Math.toRadians(270));
        }
        if(!isRed && droneSide) { // blue/drone
            initialPos = new Pose2d(-35, 60, Math.toRadians(90));
            DPL2StartPos = new Pose2d(-34, 30, Math.toRadians(0));
            DPR2StartPos = new Pose2d(-37, 30, Math.toRadians(180));
            bridgeStartPos = new Pose2d(-35, 7, Math.toRadians(180));
            DL1StartPos = new Pose2d(47, 35, Math.toRadians(0));
            DL2StartPos = new Pose2d(-60, 11.5, Math.toRadians(0));
        }
        if(!isRed && !droneSide) { // blue/stage
            initialPos = new Pose2d(10, 60, Math.toRadians(90));
        }

        robot.runWithEncoders();

        robot.resetEncoders();
        robot.runWithEncoders();

        gamepad1.reset();

//        initColorPortal();

//        position = propPosition();

//        initAprilTagPortal();
//        initColorPortal();






//        Instead of waitForStart(); :
        while(!isStarted() && !isStopRequested()){
            telemetry.addLine().addData("POSITION:", bluePosition());
            telemetry.addLine().addData("X VALUE: ", robot.getX());
            telemetry.addLine().addData("POSITION:", (bluePosition()==0) ? "Left": (bluePosition()==1) ? "Center": "Right");
            telemetry.update();
        }

        elapsedTime = System.nanoTime() - startTime;

        position = bluePosition();



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


            Pose2d startPose = new Pose2d(10,60, Math.toRadians(90));

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
                .strafeRight(35)
                .build();

            drive.followTrajectory(step2);

            pathCompleted = true;

            GlobalVariables.setAutoHeading(robot.getHeading());

            telemetry.addLine().addData("IMU HEADING: ", robot.getHeading());
            telemetry.update();

//            drive.setPoseEstimate(endPose);

//            // deposit
//            startTime1 = System.nanoTime();
//            depositTime1 = System.nanoTime() - startTime;
//
//            while((depositTime1 / 1000000000) <= 3) {
//                robot.setIntakeMotorPower(1.0);
//                depositTime1 = System.nanoTime() - startTime1;
//            }
//
//            Trajectory step3 = drive.trajectoryBuilder(endPose)
//                    .back(2)
//                    .build();
//
//            drive.followTrajectory(step3);
//            endPose = step3.end();
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
//            endPose = step3.end();



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


    public int bluePosition() {
        if(robot.getX() >= 0 && robot.getX()<= 375){
            return 1; //Center Position
        }
        else if(robot.getX() >= 425 && robot.getX()<= 625){
            return 2; //Right Position
        }
        else if(robot.getX() >= 900){
            return 0; //Left Position
        }
        else{
            return 0;
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
