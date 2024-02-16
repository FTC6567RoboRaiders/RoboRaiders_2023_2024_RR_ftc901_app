package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import RoboRaiders.Pipelines.GripPipelineBlue;
import RoboRaiders.Pipelines.StevesPipeline2;

@Autonomous(name = "Camera Auto Test")

public class CameraAutoTest extends LinearOpMode {
    OpenCvCamera camera;
    public WebcamName webcam1;
    OpenCvInternalCamera camera1;


    int cameraMonitorViewId;
    StevesPipeline2 myPipeLine;




    // UNITS ARE METERS
    double tagsize = 0.166;


    @Override
    public void runOpMode() throws InterruptedException{
        myPipeLine = new StevesPipeline2();

        webcam1 = hardwareMap.get(WebcamName .class, "Webcam 1");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(myPipeLine);

        camera.openCameraDeviceAsync(new  OpenCvCamera.AsyncCameraOpenListener()


        {
            @Override
            public void onOpened()
            {
 //               camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
 //               camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                camera.startStreaming(1280,960,OpenCvCameraRotation.UPRIGHT);



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

        //        Instead of waitForStart(); :
        while(!isStarted() && !isStopRequested()) {

//            telemetry.addLine().addData("POSITION:", bluePosition());
//            telemetry.addLine().addData("X VALUE: ", robot.getX());
//            telemetry.addLine().addData("POSITION:", (bluePosition()==0) ? "Left": (bluePosition()==1) ? "Center": "Right");
//            telemetry.update();
        }

    }


}
