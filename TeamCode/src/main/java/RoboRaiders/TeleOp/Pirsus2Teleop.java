package RoboRaiders.TeleOp;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import RoboRaiders.Robots.GlobalVariables;
import RoboRaiders.Robots.Pirsus2;
import RoboRaiders.Utilities.Logger.Logger;


// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name = "Pirsus2 Teleop")

public class Pirsus2Teleop extends OpMode {

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
    public boolean lBumperD;
    public boolean rBumperD;
    public boolean isRC = false;

    //Timer
    public long startTime;
    public long elapsedTime;
    public boolean endGame = false;  //This checks whether we have elapsed enough time to be in endgame
    public boolean bButtonG;
    public boolean lBumperG;
    public boolean rBumperG;

    public boolean liftLocked = false;

    //Lift
    public double rStickG; //Right Stick Y Value
    public boolean yButton; // Slow Lift for Hang
    public boolean rStickPress; //Press down on right stick for hold

    public double botHeading;



    public static double heading;
    public double autoHeading;






    @Override
    public void init() {

        // initialise robot and tell user that the robot is initialised
        robot.initialize(hardwareMap);


        autoHeading = GlobalVariables.getAutoHeading();
        telemetry.addLine().addData("Robot Initialized waiting your command", true);
        telemetry.addLine().addData("Heading: ", GlobalVariables.getAutoHeading());
        telemetry.addLine().addData("IMU HEADING:",  String.valueOf(botHeading));
        telemetry.addLine().addData("POSITION:", bluePosition());
        telemetry.addLine().addData("POSITION:", (bluePosition()==0) ? "Left": (bluePosition()==1) ? "Center": "Right");
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

        lBumperD = gamepad1.left_bumper;
        rBumperD = gamepad1.right_bumper;

        bButtonG = gamepad2.b;
        lBumperG = gamepad2.left_bumper;
        rBumperG = gamepad2.right_bumper;

        xButton = gamepad2.x;
        aButton = gamepad2.a;
        yButton = gamepad2.y;

        rStickG = gamepad2.right_stick_y;
        rStickPress = gamepad2.right_stick_button;




        elapsedTime = System.nanoTime() - startTime;

        // So the last 30 seconds of the 2 minute teleop (driver controlled) portion of the game
        // is referred to as endgame.  Once the timer is at 90 seconds and beyond, indicate that
        // the robot is in endGame
        if ((elapsedTime / 1000000000) >= 90) {
            endGame = true;     // Robot is in endgame
        }


//        doDriveFC();
//
//        if(lBumperD && rBumperD && !isRC){
//            isRC = true;
//        }
//        else if(lBumperD && rBumperD && isRC){
//            isRC = false;
//        }
//        if(isRC){
//            doDriveFC();
//        }
//        else{

//    }


//        }
        // doDriveRC();

        // gunner
        doLift();
        doDoor();
        doDroneLaunch();
        doIntake();
        doFlip();
        doLazySusan();
        doFlippers();
        doLiftLock();
        doDriveRC();

    }

    public void doDriveFC() {

        botHeading = robot.getHeading() + autoHeading;

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!` | PK Qual inv strafe: pos
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing | PK Qual inv strafe: pos
        double rx = gamepad1.right_stick_x; // PK Qual inv strafe: pos

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lFPower = (rotY + rotX + rx) / denominator;
        rFPower = (rotY - rotX - rx) / denominator;
        lRPower = (rotY - rotX + rx) / denominator;
        rRPower = (rotY + rotX - rx) / denominator;

        telemetry.addLine().addData("doDriveFC","doDriveFC");
        telemetry.addLine().addData("isRC: ", isRC);
        telemetry.addLine().addData("lFPower: ",  lFPower);
        telemetry.addLine().addData("rFPower: ",  rFPower);
        telemetry.addLine().addData("lRPower: ",  lRPower);
        telemetry.addLine().addData("rRPower: ",  rRPower);
        telemetry.addLine().addData("x, y, rx: ",  String.valueOf(x) + ", " + String.valueOf(y) + ", " + String.valueOf(rx));
        telemetry.addLine().addData("rotX, rotY: ",  String.valueOf(rotX) + ", " + String.valueOf(rotY));
        telemetry.addLine().addData("IMU HEADING:",  String.valueOf(botHeading));

        telemetry.update();


        robot.setDriveMotorPower(lFPower, rFPower, lRPower, rRPower);

    }

    public void doDriveRC() {

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!` | PK Qual inv strafe: pos
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing | PK Qual inv strafe: pos
        double rx = gamepad1.right_stick_x; // PK Qual inv strafe: pos

        double lFPower = y + x + rx;
        double lRPower = y - x + rx;
        double rFPower = y - x - rx;
        double rRPower = y + x - rx;

        telemetry.addLine().addData("doDriveRC","doDriveRC");
        telemetry.addLine().addData("isRC: ", isRC);
        telemetry.addLine().addData("lFPower: ",  lFPower);
        telemetry.addLine().addData("rFPower: ",  rFPower);
        telemetry.addLine().addData("lRPower: ",  lRPower);
        telemetry.addLine().addData("rRPower: ",  rRPower);
        telemetry.addLine().addData("x, y, rx: ",  String.valueOf(x) + ", " + String.valueOf(y) + ", " + String.valueOf(rx));
        telemetry.addLine().addData("IMU HEADING:",  String.valueOf(botHeading));
        telemetry.addLine().addData("POSITION:", bluePosition());
        telemetry.addLine().addData("POSITION:", (bluePosition()==0) ? "Left": (bluePosition()==1) ? "Center": "Right");
        telemetry.update();

        robot.setDriveMotorPower(lFPower, rFPower, lRPower, rRPower);

    }

    public void doIntake() {

        if(rTriggerG > 0.0) {
            robot.setIntakeMotorPower(-rTriggerG);

            robot.useLift(-.25);
        }
        else if(lTriggerG > 0.0) {

            robot.setIntakeMotorPower(lTriggerG);
        }
        else {

            robot.setIntakeMotorPower(0.0);
        }

    }

    public void doFlippers() {

        if(!bButtonG && lBumperG) { // if not B and left bumper, left flipper will trigger since B and left bumper causes airplane launch
            robot.leftFlipper(0.0);
        }
        if(rBumperG) {
            robot.rightFlipper(0.8);
        }
        if(!lBumperG) {
            robot.leftFlipper(1.0);
        }
        if(!rBumperG) {
            robot.rightFlipper(0.0);
        }

    }

    public void doDroneLaunch() {

        // As a safety measure only allow te drone launch mechanism to function during endgame AND
        // when the gamepad2 B button is pushed AND the gamepad 2 left bumper is pushed
        if (bButtonG && lBumperG) {
            robot.fireDrone(1.0);    // cleared for takeoff - roger!!
        }
        else {
            robot.fireDrone(0.0);
        }

    }

    public void doLift() {

        if(rStickG > 0.0 && !rStickPress) {
            rStickG = 1.0;
        }
        else if(rStickG < 0.0&& !rStickPress) {
            rStickG = -1.0;
        }
        else{
            rStickG = 0.0;
        }
        if(rStickPress) {
            rStickG = .1;
        }
//        if(yButton) { //Add back in endgame
//            rStickG = -0.65;
//        }

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
        if(dpadU) {
            robot.setFlipPosition(0.3, 0.2);
        }
        if(dpadD) {
            robot.setFlipPosition(1.0, .8);
            robot.setLazySusan(0.5);
            robot.setDoor(0.0);
        }
    }

    public void doLazySusan() {
        if(dpadR) {
            robot.setFlipPosition(0.145, 0.2);
            robot.setLazySusan(0.2);
        }
        if(dpadL) {
            robot.setFlipPosition(0.145, 0.2);
            robot.setLazySusan(.75);
        }

    }

    public void doLiftLock() { //This makes it so it will do the hang without us continuing to hold
        if(yButton) {
            liftLocked = true;
        }
        if(liftLocked) {
            robot.useLift(-0.65);
        }
    }

    public int bluePosition() {
        if(robot.getX() >= 0 && robot.getX()<= 375){
            return 1; //Center Position
        }
        else if(robot.getX() >= 425 && robot.getX()<= 625){
            return 2; //Right Position
        }
        else{
            return 0; //Left Position
        }

    }




}