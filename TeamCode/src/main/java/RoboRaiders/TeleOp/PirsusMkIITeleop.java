package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.Pirsus2;



// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name = "Pirsus2 Teleop")

public class PirsusMkIITeleop extends OpMode {

    public Pirsus2 robot = new Pirsus2();

    public double lFPower;
    public double rFPower;
    public double lRPower;
    public double rRPower;

    public double lTriggerG;
    public double rTriggerG;
    public boolean dpadR;
    public boolean dpadL;
    public boolean dpadU;
    public boolean dpadD;
    public boolean xButton;
    public boolean aButton;

    //Timer
    public long startTime;
    public long elapsedTime;
    public boolean endGame = false;  //This checks whether we have elapsed enough time to be in endgame
    public boolean bButtonG;
    public boolean lBumperG;

    //Lift
    public double rStickG;

    public double botHeading;


    //Flippers
    public boolean leftBumper;
    public boolean rightBumper;






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

        dpadR = gamepad2.dpad_right;
        dpadL = gamepad2.dpad_left;
        dpadU = gamepad2.dpad_up;
        dpadD = gamepad2.dpad_down;

        bButtonG = gamepad2.b;
        lBumperG = gamepad2.left_bumper;

        xButton = gamepad2.x;
        aButton = gamepad2.a;

        rStickG = gamepad2.right_stick_y;

        leftBumper = gamepad2.left_bumper;
        rightBumper = gamepad2.right_bumper;

        elapsedTime = System.nanoTime() - startTime;

        // So the last 30 seconds of the 2 minute teleop (driver controlled) portion of the game
        // is referred to as endgame.  Once the timer is at 90 seconds and beyond, indicate that
        // the robot is in endGame
        if((elapsedTime / 1000000000) >= 90) {
            endGame = true;     // Robot is in endgame
        }



        // driver
        doDrive();

        // gunner
        doLift();
        doDoor();
        doDroneLaunch();
        doIntake();
        doFlip();
        doLazySusan();
//        doFlippers();

    }

    public void doDrive() {

        botHeading = robot.getHeading();

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!`
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lFPower = (rotY + rotX + rx) / denominator;
        rFPower = (rotY - rotX - rx) / denominator;
        lRPower = (rotY - rotX + rx) / denominator;
        rRPower = (rotY + rotX - rx) / denominator;

        telemetry.addLine().addData("lFPower: ",  lFPower);
        telemetry.addLine().addData("rFPower: ",  rFPower);
        telemetry.addLine().addData("lRPower: ",  lRPower);
        telemetry.addLine().addData("rRPower: ",  rRPower);
        telemetry.addLine().addData("x, y, rx: ",  String.valueOf(x) + ", " + String.valueOf(y) + ", " + String.valueOf(rx));
        telemetry.addLine().addData("rotX, rotY: ",  String.valueOf(rotX) + ", " + String.valueOf(rotY));
        telemetry.addLine().addData("IMU HEADING:",  robot.getHeading());
        telemetry.update();


        robot.setDriveMotorPower(lFPower, rFPower, lRPower, rRPower);

    }

    public void doIntake() {

        if(rTriggerG > 0.0) {
            robot.setIntakeMotorPower(rTriggerG);
            robot.useLift(-.25);
        }
        else if(lTriggerG > 0.0) {
            robot.useLift(0.0);
            robot.setIntakeMotorPower(-lTriggerG);
        }
        else {
            robot.useLift(0.0);
            robot.setIntakeMotorPower(0.0);
        }

    }

    public void doFlippers() {

        if(!bButtonG && leftBumper) { // if not B and left bumper, left flipper will trigger since B and left bumper causes airplane launch
            robot.leftFlipper(1.0);
        }
        if(rightBumper) {
            robot.rightFlipper(1.0);
        }
        if(!leftBumper) {
            robot.leftFlipper(0.0);
        }
        if(!rightBumper) {
            robot.rightFlipper(0.0);
        }

    }

    public void doDroneLaunch() {

        // As a safety measure only allow te drone launch mechanism to function during endgame AND
        // when the gamepad2 B button is pushed AND the gamepad 2 left bumper is pushed
        if (endGame && bButtonG && lBumperG) {
            robot.fireDrone(1.0);    // cleared for takeoff - roger!!
        }
        else {
            robot.fireDrone(0.0);
        }

    }

    public void doLift() {

        if(rStickG > 1.0) {
            rStickG = 1.0;
        }
        else if(rStickG < -1.0) {
            rStickG = -1.0;
        }

        robot.useLift(rStickG);

    }



    // fix these values later
    public void doDoor() {

        if(xButton) {
            robot.setDoor(0.0);
        }
        else if(aButton) {
            robot.setDoor(1.0);
        }

    }

    public void doFlip() {
        if(dpadU){
            robot.setFlipPosition(0.2);
        }
        if(dpadD){
            robot.setFlipPosition(0.8);
            robot.setLazySusan(0.5);
            robot.setDoor(0.0);
        }
    }

    public void doLazySusan() {
        if(dpadL){
            robot.setLazySusan(0.0);
        }
        if(dpadR){
            robot.setLazySusan(1.0);
        }

    }




}