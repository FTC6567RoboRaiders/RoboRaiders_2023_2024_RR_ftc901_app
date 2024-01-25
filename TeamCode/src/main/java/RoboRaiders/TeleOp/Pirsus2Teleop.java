package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.Pirsus;


// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name = "Pirsus2 Teleop")

public class Pirsus2Teleop extends OpMode {
    public Pirsus robot = new Pirsus();


    public double lTriggerG;
    public double rTriggerG;

    //Timer
    public long startTime;
    public long elapsedTime;
    public boolean endGame = false;  //This checks whether we have elapsed enough time to be in endgame




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


    }
    @Override
    public void loop() {

        rTriggerG = gamepad2.right_trigger;
        lTriggerG = gamepad2.left_trigger;

    }

    public void doIntake(){

        if(rTriggerG > 0.0) {
            robot.setIntakeMotorPower(rTriggerG);
        }

        else if(lTriggerG > 0.0) {
            robot.setIntakeMotorPower(-lTriggerG);
        }

        else {
            robot.setIntakeMotorPower(0.0);
        }
    }

}

