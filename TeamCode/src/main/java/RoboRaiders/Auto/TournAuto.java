package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
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
    public int position;
    public long startTime;
    public long elapsedTime;

    private static final boolean USE_WEBCAM = true;

    @Override
    public void runOpMode() {

        robot.initialize(hardwareMap);

        robot.runWithEncoders();

        robot.resetEncoders();
        robot.runWithEncoders();

        gamepad1.reset();

        initColourPortal();

        position = propPosition();

        waitForStart();

        elapsedTime = System.nanoTime() - startTime;

        // spike mark positions
        switch (position) {
            // move to left spikemark
            case 0:
                break;
            // move to centre spikemark
            case 1:
                break;
            // move to right spikemark
            case 2:
                break;
            default:
                break;
        }

        initAprilTagPortal();

        // intake and deposit loop
        while((elapsedTime / 1000000000) >= 5) {



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

    public void initColourPortal() {

        // how?

    }

    public int propPosition() {

        if(stevesPipeline.returnX() <= 319) {
            return 1;
        }
        else if(stevesPipeline.returnX() >= 320) {
            return 2;
        }
        else {
            return 0;
        }

    }

}
