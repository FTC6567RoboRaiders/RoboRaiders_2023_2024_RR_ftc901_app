package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.MatOfPoint;


import java.util.ArrayList;

import RoboRaiders.Robots.CameraBot;


@Autonomous(name = "Camera")

public class cameraTeleop extends OpMode {

    public CameraBot robot = new CameraBot();

    @Override
    public void init() {

        // initialise robot and tell user that the robot is initialized
        robot.initialize(hardwareMap);

    }

    @Override
    public void start() {



    }

    @Override
    public void loop() {

        doCamera();

        // End code added by Steeeve
    }



    public void doCamera() {
//        ArrayList<MatOfPoint> foundContoursBlue = robot.stevesPipeline.getFoundContoursBlue();
//        ArrayList<MatOfPoint> foundContoursRed = robot.stevesPipeline.getFoundContoursRed();

        // more work to do here now


    }

}
