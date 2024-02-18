package RoboRaiders.Robots;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import RoboRaiders.Pipelines.StevesPipeline2;
import RoboRaiders.Tests.RRBlinkinDriver;


public class CameraBot {

    public int xCoord;
    public int yCoord;
    public int xBCoord;
    public int yBCoord;

    public HardwareMap hwMap = null;
    OpenCvCamera camera;
    int cameraMonitorViewId;

    public WebcamName webcam1;
    // Vision Variables
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    NormalizedColorSensor normalizedColorSensor;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    public int relativeLayoutId;

    //RevBlinkinLedDriver blinkinLedDriver;
    RRBlinkinDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    RevBlinkinLedDriver.BlinkinPattern[] elements = RevBlinkinLedDriver.BlinkinPattern.values();

    public CameraBot() {

    }


    public void initialize(HardwareMap ahwMap) {

        // save reference to hardware map
        hwMap = ahwMap;

    }

    public void setX(int xVal){
        xCoord = xVal;

    }
    public void setY(int yVal){
        yCoord = yVal;
    }
    public void setBX(int xBVal){
        xBCoord = xBVal;
    }
    public void setBY(int yBVal){
        yBCoord = yBVal;
    }

    public int getX(){
        return xCoord;
    }
    public int getY(){
        return yCoord;
    }
    public int getBX(){
        return xBCoord;
    }
    public int getBY(){
        return yBCoord;
    }

}

