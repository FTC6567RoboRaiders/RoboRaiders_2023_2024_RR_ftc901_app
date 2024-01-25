package RoboRaiders.Tests;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import java.util.Locale;

import RoboRaiders.Robots.Hubbot;


@Autonomous(name = "HubBot")

public class HubAuto extends OpMode {

    public static final int[] PURPLE = { 195, 210 };
    public static final int[] YELLOW = { 80, 110 };
    public static final int[] GREEN = { 125, 145 };
    public static final int[] WHITE = { 155, 170 };

    public Hubbot robot = new Hubbot();

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
        doColor();

        // End code added by Steeeve
    }

    @Override
    public void stop() {
       // robot.resetBackground();
    }



    public void doCamera() {
//        ArrayList<MatOfPoint> foundContoursBlue = robot.stevesPipeline.getFoundContoursBlue();
//        ArrayList<MatOfPoint> foundContoursRed = robot.stevesPipeline.getFoundContoursRed();

        // more work to do here now


    }
    public void doColor() {
        telemetry.addData("**** REV V3 Color Data","****");
        telemetry.addData("Distance (in)",
                String.format(Locale.US, "%.02f", robot.getDistance()));
        telemetry.addData("Alpha", robot.getAlpha());

        int [] rgb = robot.getRGB();
        telemetry.addLine()
                 .addData("Red  ", rgb[0])
                 .addData("Green", rgb[1])
                 .addData("Blue ", rgb[2]);

        float [] hsvValues = robot.getHSV();
        telemetry.addLine()
                 .addData("Hue", hsvValues[0])
                 .addData("Saturation ", hsvValues[1])
                 .addData("Value ", hsvValues[2]);


        // Get normalized colors - will it make a difference curious minds want to know
        NormalizedRGBA colors = robot.getNormalizedColors();
        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addData("**** Normalized Colors ", "****");
        telemetry.addLine()
                 .addData("Red", "%.3f", colors.red)
                 .addData("Green", "%.3f", colors.green)
                 .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                 .addData("Hue", "%.3f", hsvValues[0])
                 .addData("Saturation", "%.3f", hsvValues[1])
                 .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);


        telemetry.update();
        //robot.changeBackground(hsvValues);

        if ((int)hsvValues[0] >= PURPLE[0] && (int)hsvValues[0] <= PURPLE[1]) robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        else
            if ((int)hsvValues[0] >= YELLOW[0] && (int)hsvValues[0] <= YELLOW[1]) robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            else
                if ((int)hsvValues[0] >= GREEN[0] &&(int)hsvValues[0] <= GREEN[1]) robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                else
                    if ((int)hsvValues[0] >= WHITE[0] && (int)hsvValues[0] <= WHITE[1]) robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    else robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

    }

}
