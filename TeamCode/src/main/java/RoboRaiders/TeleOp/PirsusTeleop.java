package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.PirsusMkII;
import RoboRaiders.Utilities.Logger.Logger;

@TeleOp (name = "Pirsus TeleOp")
@Disabled

public class PirsusTeleop extends OpMode {

    public PirsusMkII robot = new PirsusMkII();
    public Logger myLogger =  new Logger("TestBotTeleop");
    public Logger dtLogger = new Logger("DT");   // Drive train logger

    public long startTime;
    public long elapsedTime;
    public boolean endGame = false;

    @Override
    public void init() {
        // initialise robot and tell user that the robot is initialised
        robot.initialize(hardwareMap);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();
    }

    @Override
    public void start() {
        //Timer for drone launch safety
        startTime = System.nanoTime();
    }

    @Override
    public void loop() {



    }

}
