package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.Pirsus2;

@TeleOp (name = "Reset Purple Pixel")


public class spitPurplePixel extends OpMode{

    public Pirsus2 robot = new Pirsus2();


    public boolean xButton;


    //Timer
    public long startTime;
    public long elapsedTime;






    @Override
    public void init() {

        // initialise robot and tell user that the robot is initialised
        robot.initialize(hardwareMap);
        robot.setElbowPosition(1.0,0.8);
        robot.setLazySusan(0.5);
        robot.setDoor(0.0);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();


    }

    @Override
    public void start() {



    }

    @Override
    public void loop() {

        telemetry.addLine().addData("Press A to run outtake for 2 seconds", true);
        telemetry.update();
        xButton = gamepad2.x;
        elapsedTime = 0;
        if(xButton){
            startTime = System.nanoTime();

            while((elapsedTime / 1000000000) < 2) {
                elapsedTime = System.nanoTime() - startTime;
                spitPixel();
            }
        }
        robot.setIntakeMotorPower(0.0);






    }

    public void spitPixel(){
        robot.setIntakeMotorPower(1.0);
    }









}
