package RoboRaiders.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




public class LiftBot {

    public DcMotorEx leftLiftMotor = null;
    public DcMotorEx rightLiftMotor = null;
    public HardwareMap hwMap = null;

    public IMU imu;

    public IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)
    );

    public Orientation angles;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

    public static double robotHeading;
    public boolean firstTimeCalled = true;


    public LiftBot() {

    }


    public void initialize(HardwareMap ahwMap) {

        // save reference to hardware map
        hwMap = ahwMap;

        // define and initialize motors
        leftLiftMotor  = hwMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLiftMotor = hwMap.get(DcMotorEx.class, "rightLiftMotor");

        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.setPower(0.0);
        rightLiftMotor.setPower(0.0);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize sensors
        imu = hwMap.get(IMU.class, "imu");
//        parameters.angleUnit = IMU.AngleUnit.RADIANS;
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

    }


    /**
     * This method will set the power for the drive motors
     *
     * @param left  power setting for the left motor
     * @param right power setting for the right motor
     */
    public void setLiftMotorPower(double left, double right) {
        leftLiftMotor.setPower(left);
        rightLiftMotor.setPower(right);
    }

    public double liftCalculateCounts(double distance) {
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

    public int getAverageEncoderCount() {
        double[] encoderArray = new double[2];

        encoderArray[0] = Math.abs(getLeftLiftMotorEncoderCounts());
        encoderArray[1] = Math.abs(getRightLiftMotorEncoderCounts());

        int averageCount = (int) (encoderArray[0] + encoderArray[1]) / 2;

        return averageCount;
    }

    public void setDTMotorTargetPosition(int encoderPosition) {
        leftLiftMotor.setTargetPosition(encoderPosition);
        rightLiftMotor.setTargetPosition(encoderPosition);
    }

    public void runWithEncoders() {
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithEncodersSTP() {
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runWithoutEncoders() {
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders() {
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getLeftLiftMotorEncoderCounts()  { return leftLiftMotor.getCurrentPosition();  }
    public double getRightLiftMotorEncoderCounts() { return rightLiftMotor.getCurrentPosition(); }



    public float getHeading() {

        float heading;

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); // heading is equal to the absolute value of the first angle
//        heading = Math.abs(imu.getRobotOrientation().firstAngle);

        return heading;
    }


    public void resetIMU() {

        imu.initialize(parameters);
    }


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

}
