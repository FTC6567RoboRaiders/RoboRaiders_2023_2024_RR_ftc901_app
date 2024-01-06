package RoboRaiders.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import RoboRaiders.Pipelines.StevesPipeline;
import RoboRaiders.Utilities.Logger.Logger;
import RoboRaiders.Pipelines.GripPipelineRed;
import RoboRaiders.Pipelines.GripPipelineBlue;



public class CameraBot {

    public HardwareMap hwMap = null;
    OpenCvCamera camera;
    int cameraMonitorViewId;

    public WebcamName webcam1;
    // Vision Variables
    public StevesPipeline stevesPipeline;

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

        stevesPipeline = new StevesPipeline();
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

