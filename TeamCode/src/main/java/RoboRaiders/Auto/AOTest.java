package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import RoboRaiders.Auto.AutoOptions.AutoOptions;
import RoboRaiders.Pipelines.GripPipelineBlue;
import RoboRaiders.Pipelines.GripPipelineRed;
import RoboRaiders.Robots.CameraBot;
import RoboRaiders.Robots.Hubbot;


@Autonomous


public class AOTest extends LinearOpMode {

    public CameraBot robot = new CameraBot();
    AutoOptions AO = new AutoOptions(this);

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    GripPipelineRed redPipeline;
    GripPipelineBlue bluePipeline;


    OpenCvCamera camera;
    public WebcamName webcam1;

    int cameraMonitorViewId;

    private boolean isRed = false;
    private boolean stageSide = false;
    private boolean waitForPartner = false;

    private boolean selectionsAreGood = false;

    double numOfTicks;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int waitTime = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        while (!selectionsAreGood) {



            isRed = AO.selectAlliance();              // red or blue
            stageSide = AO.selectStartLocation();         // starting near the drones or the backboard
            waitForPartner = AO.selectWait();                   // wait for partner
            if(waitForPartner){
                waitTime = AO.selectWaitTime();
            }
            // Add new/additional auto options, so things like drive to depot, drop team marker, etc..


            // Display the options selected
            // We show two options per line, to save space and lines.  The maximum number of characters
            // per line is roughly 45.  Maximum number of lines to be displayed is 9.
            // Note: To keep the autonomous options displayed, the automagical clearing of the telemetry data will be
            //       turned off with the setAutoClear(false) prior to calling selectionsGood().  After selectionsGood()
            //       turn on the automagical clearing of the telemetry data which is the default action.
            telemetry.setAutoClear(false);
            telemetry.addLine().addData("Autonomous", "Selections");
            telemetry.addLine().addData("Alliance:", isRed ? "Red  " : "Blue  ").addData("  Robot Start Location:", stageSide ? "Stage" : "Back Stage");
            telemetry.addLine().addData("Wait for Partner:", waitForPartner ? "Yes" : "No");
            if(waitForPartner) {
                telemetry.addLine().addData("Wait Time: ", waitTime);
            }
            telemetry.update();

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, we'll ask again.

            selectionsAreGood = AO.selectionsGood();
            telemetry.setAutoClear(true);
            telemetry.update();    // Clear the selections
        }

        telemetry.addLine().addData("Waiting your command", true);
        telemetry.update();


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

        waitForStart();

        telemetry.addLine().addData("Selected alliance:", isRed);
        telemetry.addLine().addData("Selected side:", stageSide);
        telemetry.addLine().addData("Wait:", waitForPartner);
        if(waitForPartner){
            telemetry.addLine().addData("Wait Time: ", waitTime);

        }

        /** power controls:
         *  -, -, -, - | forward
         *  +, +, +, + | backward
         *  -, +, +, - | left
         *  +, -, -, + | right
         */

    }

}
