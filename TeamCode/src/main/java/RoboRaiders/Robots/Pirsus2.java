package RoboRaiders.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import RoboRaiders.Utilities.Logger.Logger;


public class Pirsus2 {
    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotorEx lFMotor = null;
    public DcMotorEx rFMotor = null;
    public DcMotorEx lRMotor = null;
    public DcMotorEx rRMotor = null;

    public DcMotorEx intakeMotor = null;


    public DcMotorEx launchMotor = null;



    public DcMotorEx armMotor = null;


    //Lift
    public DcMotorEx liftMotorUp = null;
    public DcMotorEx liftMotorDown = null;

    //Deposit
    public Servo flipServo = null;
    public Servo doorServo = null;
    public Servo scrubServo = null;


    public IMU imu;

    /* Local OpMode Members */
    public HardwareMap hwMap = null;

    /* Public Variables */
    public IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
    );

//    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(
//            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
//    );
    public Orientation angles;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

    public static double robotHeading;
    public boolean firstTimeCalled = true;

    /* Camera */

    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */
    public Pirsus2() {

    }

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        // save reference to hardware map
        hwMap = ahwMap;

        // define and initialize motors
        lFMotor = hwMap.get(DcMotorEx.class, "lFMotor");
        rFMotor = hwMap.get(DcMotorEx.class, "rFMotor");
        lRMotor = hwMap.get(DcMotorEx.class, "lRMotor");
        rRMotor = hwMap.get(DcMotorEx.class, "rRMotor");

        // intake motors
        intakeMotor = hwMap.get(DcMotorEx.class, "rIntakeMotor");

        // drone launch catch
        launchMotor = hwMap.get(DcMotorEx.class, "launchMotor");

        // intake motor and servos
        armMotor = hwMap.get(DcMotorEx.class, "armMotor");


        // lift servo
        liftMotorUp = hwMap.get(DcMotorEx.class, "liftMotorUp");
        liftMotorDown = hwMap.get(DcMotorEx.class, "liftMotorDown");

        flipServo = hwMap.get(Servo.class, "flipServo");
        scrubServo = hwMap.get(Servo.class, "scrubServo");
        doorServo = hwMap.get(Servo.class, "doorServo");



        // defines the directions the motors will spin
        lFMotor.setDirection(DcMotor.Direction.REVERSE);
        rFMotor.setDirection(DcMotor.Direction.FORWARD);
        lRMotor.setDirection(DcMotor.Direction.REVERSE);
        rRMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);


        lFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //have the motors on the drivetrain break here.
        // Set all motors to zero power
        rFMotor.setPower(0.0);
        lFMotor.setPower(0.0);
        rRMotor.setPower(0.0);
        lRMotor.setPower(0.0);

        intakeMotor.setPower(0.0);


        armMotor.setPower(0.0);


        // Stop and reset encoders
        resetEncoders();


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        lFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
        rFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // Define and initialize sensors
        imu = hwMap.get(IMU.class, "imu");
//        parameters.angleUnit = IMU.AngleUnit.RADIANS;
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

    }


    //**********************************************************************************************
    //
    // DRIVE TRAIN METHODS
    //
    //**********************************************************************************************

    /**
     * This method will set the power for the drive motors
     *
     * @param leftFront  power setting for the left front motor
     * @param rightFront power setting for the right front motor
     * @param leftBack   power setting for the left back motor
     * @param rightBack  power setting for the right back motor
     */
    public void setDriveMotorPower(double leftFront, double rightFront, double leftBack, double rightBack) {

        lFMotor.setPower(leftFront);
        rRMotor.setPower(rightBack);
        lRMotor.setPower(leftBack);
        rFMotor.setPower(rightFront);

    }

    /**
     * This method will set the power for the drive motors
     *
     * @param leftFront  power setting for the left front motor
     * @param rightFront po`wer setting for the right front motor
     * @param leftBack   power setting for the left back motor
     * @param rightBack  power setting for the right back motor
     */
    public void setDriveMotorPower(double leftFront, double rightFront, double leftBack, double rightBack, Logger lclLogger) {

        lFMotor.setPower(leftFront);
        rFMotor.setPower(rightFront);
        lRMotor.setPower(leftBack);
        rRMotor.setPower(rightBack);
        if (firstTimeCalled) {
            lclLogger.Debug("========================================================================================");
            lclLogger.Debug("| FIRST TIME CALLED  FIRST TIME CALLED  FIRST TIME CALLED  FIRST TIME CALLED           |");
            lclLogger.Debug("========================================================================================");
            firstTimeCalled = false;
        }



    }

    /**
     * Setting Power for the arm motor on robot
     * @param power
     */
    public void setArmMotorPower(double power){
        armMotor.setPower(power);
    }


    /**
     * calculates the number of encoder counts to travel a given distance for the drive train motors
     *
     * @param distance
     * @return
     */
    public double driveTrainCalculateCounts(double distance) {

        double COUNTS;

        int DIAMETER = 4; // diameter of wheel
        double GEAR_RATIO = (1.0 / 1.0); // gear ratio
        double PULSES = 537.7; // encoder counts in one revolution - goBilda YJPGM 19.2:1

//        double PULSES = 537.6; // encoder counts in one revolution - neverest 20 orbital
//        double PULSES = 1120.0; // encoder counts in one revolution - neverest 40 orbital
//        double PULSES = 1680.0; // encoder counts in one revolution - neverest 60 orbital

        double CIRCUMFERENCE = Math.PI * DIAMETER; // gives circumference
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; // gives rotations
        COUNTS = PULSES * ROTATIONS; // gives counts

        return COUNTS;
    }

    /**
     * Takes the four drive train encoder values and sorts them using a bubble sort algorithm from
     * lowest to highest.  Throws out the lowest and highest values in the sorted list and averages
     * the two remaining values
     *
     * @return average of the two middle encoder counts
     */
    public int getSortedEncoderCount() {

        int[] encoderArray = new int[4];

        encoderArray[0] = Math.abs(lFMotor.getCurrentPosition());
        encoderArray[1] = Math.abs(rFMotor.getCurrentPosition());
        encoderArray[2] = Math.abs(lRMotor.getCurrentPosition());
        encoderArray[3] = Math.abs(rRMotor.getCurrentPosition());

        int I;
        int J;
        int Temp;

        for (I = 0; I < 3; I++) {
            for (J = I + 1; J < 4; J++) {
                if (encoderArray[I] < encoderArray[J]) {
                } else {

                    Temp = encoderArray[I];
                    encoderArray[I] = encoderArray[J];
                    encoderArray[J] = Temp;
                }
            }
        }
        int averageCount = (encoderArray[1] + encoderArray[2]) / 2;

        return averageCount;
    }

    /**
     * Sets the target encoder value for the drive train motors
     *
     * @param encoderPosition
     */

    public void setDTMotorTargetPosition(int encoderPosition) {

        lFMotor.setTargetPosition(encoderPosition);
        rFMotor.setTargetPosition(encoderPosition);
        lRMotor.setTargetPosition(encoderPosition);
        rRMotor.setTargetPosition(encoderPosition);

    }

    /**
     * This method will set the mode of all of the drive train motors to run using encoder
     */
    public void runWithEncoders() {

        lFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    /**
     * This method will set the mode all of the drive train motors to RUN_TO_POSITION
     */
    public void runWithEncodersSTP() {

        lRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * This method will set the mode of all of the drive train motors to run without encoder
     */
    public void runWithoutEncoders() {

        lFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * This will set the mode of the drive train motors to STOP_AND_RESET_ENCODER, which will zero
     * the encoder count but also set the motors into a RUN_WITHOUT_ENCODER mode
     */
    public void resetEncoders() {

        lFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    /**
     * These methods will get individual encoder position from any of the drive train motors
     *
     * @return the encoder position
     */
    public double getBackLeftDriveEncoderCounts() {
        return lRMotor.getCurrentPosition();
    }

    public double getBackRightDriveEncoderCounts() {
        return rRMotor.getCurrentPosition();
    }

    public double getFrontLeftDriveEncoderCounts() {
        return lFMotor.getCurrentPosition();
    }

    public double getFrontRightDriveEncoderCounts() {
        return rFMotor.getCurrentPosition();
    }

    // turret motor methods

    /**
     * Gets the current heading from the IMU
     *
     * @return current heading in degrees
     */
    public float getHeading() {

        float heading;

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); // heading is equal to the absolute value of the first angle
//        heading = Math.abs(imu.getRobotOrientation().firstAngle);

        return heading;
    }

    /**
     * re-initializes the IMU
     */
    public void resetIMU() {

        imu.initialize(parameters);
    }

    /**
     * calculates and returns an integrate Z-Axis (aka heading), this handles when the heading crosses
     * 180 or -180
     *
     * @return integrated Z-Axis
     */
    public double getIntegratedZAxis() {

        // This sets up the how we want the IMU to report data
        iza_angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Obtain the heading (Z-Axis)
        iza_newHeading = iza_angles.firstAngle;

        // Calculate the change in the heading from the previous heading
        iza_deltaHeading = iza_newHeading - iza_lastHeading;

        // Bosch IMU wraps at 180, so compensate for this
        if (iza_deltaHeading <= -180.0) {

            iza_deltaHeading += 360.0;
        } else if (iza_deltaHeading >= 180.0) {

            iza_deltaHeading -= 360;
        }

        // Calculate the integratedZAxis
        integratedZAxis += iza_deltaHeading;

        // Save the current heading for the next call to this method
        iza_lastHeading = iza_newHeading;

        return integratedZAxis;
    }

    //**********************************************************************************************
    //
    // END IMU METHODS
    //
    //**********************************************************************************************

    //**********************************************************************************************
    //
    // GUNNER METHODS
    //
    //**********************************************************************************************

    public void runArmWithEncoders() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setIntakeMotorPower(double intakePower) {

        intakeMotor.setPower(intakePower);

    }

    public void fireDrone() {
        launchMotor.setPower(1.0);
    }

    public int getArmEncoders() {
        return armMotor.getCurrentPosition();
    }

    public void resetArmMotorEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void scrub(double scrubSide){
        scrubServo.setPosition(scrubSide);
    }

    public void doorOpen(){

    }


    public void useLift(double power) {
        liftMotorUp.setPower(power);
        liftMotorDown.setPower(-power);
    }






    //**********************************************************************************************
    //
    // END GUNNER METHODS
    //
    //**********************************************************************************************

    //**********************************************************************************************
    //
    // CAMERA METHODS
    //
    //**********************************************************************************************



}