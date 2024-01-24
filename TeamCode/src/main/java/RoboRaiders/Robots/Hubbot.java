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
import org.openftc.easyopencv.OpenCvCameraRotation;

import RoboRaiders.Pipelines.StevesPipeline2;
import RoboRaiders.Tests.RRBlinkinDriver;


public class Hubbot {

    public HardwareMap hwMap = null;
    OpenCvCamera camera;
    int cameraMonitorViewId;

    public WebcamName webcam1;
    // Vision Variables
    public StevesPipeline2 stevesPipeline;
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
    public Hubbot() {

    }


    public void initialize(HardwareMap ahwMap) {

        // save reference to hardware map
        hwMap = ahwMap;


        // Vision processing
        webcam1 = hwMap.get(WebcamName.class, "Webcam 1");
        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        camera.openCameraDevice();

//        stevesPipeline = new StevesPipeline2(this);


//        camera.openCameraDeviceAsync(new  OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened()
//            {
//                camera.setPipeline(stevesPipeline);
//                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                // For now do nothing when we have an error
//            }
//        });

        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        normalizedColorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color_distance");

        relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());

        blinkinLedDriver = hwMap.get(RRBlinkinDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        blinkinLedDriver.setPattern(pattern);

    }
    public int[] getRGB() {
        int[] myRBG = new int[3];
        myRBG[0] = sensorColor.red();
        myRBG[1] = sensorColor.green();
        myRBG[2] = sensorColor.blue();

        return myRBG;
    }

    public int getAlpha() { return sensorColor.alpha(); }

    public float[] getHSV() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues;
    }
    public NormalizedRGBA getNormalizedColors() {
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();
        return colors;
    }
    public float getDistance() { return (float) sensorDistance.getDistance(DistanceUnit.INCH); }

    public void changeBackground( float [] hsvValues ) {
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.

        final View relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hsvValues));
            }
        });
    }

    public void resetBackground() {
        final View relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern)
    {
        blinkinLedDriver.setPattern(pattern);
        this.pattern = pattern;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() { return this.pattern; }

    public ServoControllerEx getController() { return blinkinLedDriver.getController(); }
    public int getPort() { return blinkinLedDriver.getPort(); }
    public double getControllerPosition() { return blinkinLedDriver.getControllerPosition(); }
    public void setControllerPosition( double pwm ) {
        int port = this.getPort();
        ServoControllerEx controller = this.getController();
        controller.setServoPosition(port,pwm);

    }
    public RevBlinkinLedDriver.BlinkinPattern getPattern(double pwm) {
        return elements[blinkinLedDriver.getOrdinal(pwm)];
    }
    public int getOrdinal(double pwm) { return blinkinLedDriver.getOrdinal(pwm); }
}

