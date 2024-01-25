package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.Pirsus2;


// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name = "Pirsus2 Teleop")

public class Pirsus2Teleop extends OpMode {
    public Pirsus2 robot = new Pirsus2();


    public double lTriggerG;
    public double rTriggerG;

    //Timer
    public long startTime;
    public long elapsedTime;
    public boolean endGame = false;  //This checks whether we have elapsed enough time to be in endgame
    public boolean bButtonG;
    public boolean lBumperB;



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

        rTriggerG = gamepad2.right_trigger;
        lTriggerG = gamepad2.left_trigger;

        bButtonG = gamepad2.b;
        lBumperB = gamepad2.left_bumper;

        elapsedTime = System.nanoTime() - startTime;

        if((elapsedTime / 1000000000) >= 90) {
            endGame = true;
        }


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

    public void doDroneLaunch() {

        if (endGame && bButtonG && lBumperB) {
            robot.fireDroneTrigger(0.0);
        }
    }

    public void doLift(){

    }

    public void doDeposit(){

    }


}