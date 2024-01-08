package RoboRaiders.Robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import RoboRaiders.Pipelines.StevesPipeline;


public class CameraBot {

    public HardwareMap hwMap = null;
    OpenCvCamera camera;
    int cameraMonitorViewId;

    public WebcamName webcam1;
    // Vision Variables
    public StevesPipeline2 stevesPipeline;

    public CameraBot() {

    }


    public void initialize(HardwareMap ahwMap) {

        // save reference to hardware map
        hwMap = ahwMap;


        // Vision processing
        webcam1 = hwMap.get(WebcamName.class, "Webcam 1");
        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();

        stevesPipeline = new StevesPipeline2();
        camera.setPipeline(stevesPipeline);
        camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
//        camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);

//        camera.openCameraDeviceAsync(new  OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                // For now do nothing when we have an error
//            }
//        });

    }

}

