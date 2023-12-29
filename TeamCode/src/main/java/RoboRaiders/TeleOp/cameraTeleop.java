package RoboRaiders.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import RoboRaiders.Robots.NotPirsus;
import RoboRaiders.Utilities.Logger.Logger;
import RoboRaiders.Pipelines.GripPipelineRed;
import RoboRaiders.Pipelines.GripPipelineBlue;

@Autonomous(name = "Camera")

public class cameraTeleop extends OpMode {

    public NotPirsus robot = new NotPirsus();
//    public GripPipelineRed redPipeline = new GripPipelineRed();
//    public GripPipelineBlue bluePipeline = new GripPipelineBlue();

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
        ArrayList<MatOfPoint> foundCountours = robot.stevesPipeline.getFoundContours();

        // more work to do here now


    }

}
