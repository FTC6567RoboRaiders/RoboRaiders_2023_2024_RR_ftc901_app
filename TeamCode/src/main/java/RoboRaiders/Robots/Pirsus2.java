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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import RoboRaiders.Pipelines.StevesPipeline2;

import RoboRaiders.Utilities.Logger.Logger;


public class Pirsus2  extends RRRobot{


    OpenCvCamera camera;
    public WebcamName webcam1;

    int cameraMonitorViewId;

    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotorEx lFMotor = null;
    public DcMotorEx rFMotor = null;
    public DcMotorEx lRMotor = null;
    public DcMotorEx rRMotor = null;

    public DcMotorEx intakeMotor = null;


    public DcMotorEx launchMotor = null;
    public Servo launchServo = null;

    //Lift
    public DcMotorEx liftMotorRight = null;
    public DcMotorEx liftMotorLeft = null;

    //Intake Flippers We're Not Using Right Now
    public Servo flipServoL = null;
    public Servo flipServoR = null;

    public Servo doorServo = null;
    public Servo lazySusanServo = null;

    public Servo elbowServoR = null;
    public Servo elbowServoL = null;
    public Servo wristServo = null;



    public IMU imu;

    /* Local OpMode Members */
    public HardwareMap hwMap = null;

    /* Public Variables */
//    public IMU.Parameters parameters = new IMU.Parameters(
//            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
//    );

        public IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
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

    public int redX;
    public int redY;

    public int xCoord;
    public int yCoord;
    public int xBCoord;
    public int yBCoord;


    public StevesPipeline2 stevesPipeline = new StevesPipeline2(this);

    public Pirsus2() {

    }

    /* Camera */

    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */


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
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        // drone launch catch
        launchServo = hwMap.get(Servo.class, "launchServo");

        // intake motor and servos



        // lift servo
        liftMotorRight = hwMap.get(DcMotorEx.class, "liftMotorRight");
        liftMotorLeft = hwMap.get(DcMotorEx.class, "liftMotorLeft");

        flipServoL = hwMap.get(Servo.class, "flipServo1");
        flipServoR = hwMap.get(Servo.class, "flipServo2");

        doorServo = hwMap.get(Servo.class, "doorServo");
        lazySusanServo = hwMap.get(Servo.class,"lazySusanServo");
        elbowServoR = hwMap.get(Servo.class,"armServoR");
        elbowServoL = hwMap.get(Servo.class,"armServoL");
        wristServo = hwMap.get(Servo.class, "wristServo");


        setFlipPosition(1.0, 0.8);
        setLazySusan(0.5);
        setDoor(0.0);
        leftFlipper(1.0);
        rightFlipper(0.0);
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

////         Vision processing
//        webcam1 = hwMap.get(WebcamName.class, "Webcam 1");
//        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        camera.setPipeline(stevesPipeline);
//
//        camera.openCameraDeviceAsync(new  OpenCvCamera.AsyncCameraOpenListener()
//
//
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
//                Logger Log = new Logger(String.valueOf("******** ON OPENED *******"));
//                Log.Debug("X COORDINATE: ", getX());
//                Log.Debug("Y COORDINATE: ", getY());
//                Log.Debug("X 'Bottom COORDINATE: ", getBX());
//                Log.Debug("Y Bottom COORDINATE: ", getBY());
//
//
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                // For now do nothing when we have an error
//            }
//
//        });




        //have the motors on the drivetrain break here.
        // Set all motors to zero power
        rFMotor.setPower(0.0);
        lFMotor.setPower(0.0);
        rRMotor.setPower(0.0);
        lRMotor.setPower(0.0);

        intakeMotor.setPower(0.0);








        // Stop and reset encoders
        resetEncoders();


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        lFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
        rFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);







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

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // this sets up the how we want the IMU to report data
        heading = angles.firstAngle; // heading is equal to the absolute value of the first angle
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


    public void setIntakeMotorPower(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    // flywheel drone launcher
//    public void fireDrone(double power) {
//        launchMotor.setPower(power);
//    }

    // servo drone launcher
    public void fireDrone(double position) {
        launchServo.setPosition(position);
    }

    public void leftFlipper(double pos) {
        flipServoL.setPosition(pos);
    }

    public void rightFlipper(double pos) {
        flipServoR.setPosition(pos);
    }


    public void useLift(double power) {
        liftMotorRight.setPower(power * 0.75);
        liftMotorLeft.setPower(-power * 0.75);
    }

    public void setFlipPosition(double posL, double posR) {
//        armServoR.setPosition(pos);
        elbowServoL.setPosition(posL);
//        armServoR.setPosition(posR);
    }
    public void setLazySusan(double pos) {
        lazySusanServo.setPosition(pos);
    }

    public void setDoor(double pos) {
        doorServo.setPosition(pos);
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
    public int[] getRed(){
        int[] vals = new int[]{redX, redY};
        return vals;
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
    public int positionBlue(){
        return 0;
    }

}