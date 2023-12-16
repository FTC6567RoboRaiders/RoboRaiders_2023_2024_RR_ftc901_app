package RoboRaiders.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

import RoboRaiders.Utilities.Logger.Logger;



public class NotPirsus {

    public DcMotorEx lFMotor = null;
    public DcMotorEx rFMotor = null;
    public DcMotorEx lRMotor = null;
    public DcMotorEx rRMotor = null;

    public DcMotorEx lDeadwheel = null;
    public DcMotorEx rDeadwheel = null;
    public DcMotorEx bDeadwheel = null;

    public HardwareMap hwMap = null;

    public IMU imu;

    public IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
    );

    public Orientation angles;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

    public static double robotHeading;
    public boolean firstTimeCalled = true;


    public NotPirsus() {

    }


    public void initialize(HardwareMap ahwMap) {

        // save reference to hardware map
        hwMap = ahwMap;

        // define and initialize motors
        lFMotor = hwMap.get(DcMotorEx.class, "lFMotor");
        rFMotor = hwMap.get(DcMotorEx.class, "rFMotor");
        lRMotor = hwMap.get(DcMotorEx.class, "lRMotor");
        rRMotor = hwMap.get(DcMotorEx.class, "rRMotor");

        lDeadwheel = hwMap.get(DcMotorEx.class, "lDeadwheel");
        rDeadwheel = hwMap.get(DcMotorEx.class, "rDeadwheel");
        bDeadwheel = hwMap.get(DcMotorEx.class, "bDeadwheel");


        lFMotor.setDirection(DcMotor.Direction.REVERSE);
        rFMotor.setDirection(DcMotor.Direction.FORWARD);
        lRMotor.setDirection(DcMotor.Direction.REVERSE);
        rRMotor.setDirection(DcMotor.Direction.FORWARD);

        lFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rFMotor.setPower(0.0);
        lFMotor.setPower(0.0);
        rRMotor.setPower(0.0);
        lRMotor.setPower(0.0);


        lFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize sensors
        imu = hwMap.get(IMU.class, "imu");
//        parameters.angleUnit = IMU.AngleUnit.RADIANS;
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

    }


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
     * @param rightFront power setting for the right front motor
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

        if (leftFront != 0.0 && rightFront != 0.0 && leftBack != 0.0 && rightBack != 0.0) {
            lclLogger.Debug("************* TestRobot Set Drive Motor Power TestRobot Set Drive Motor Power **********");
            lclLogger.Debug("DT Motor Powers       (LF, RF, LB, RB): ", leftFront, rightFront, leftBack, rightBack);
            lclLogger.Debug("Retrieved DT Powers   (LF, RF, LB, RB): ", lFMotor.getPower(), rFMotor.getPower(), lRMotor.getPower(), rRMotor.getPower());
            lclLogger.Debug("Retrieved DT Currents (LF, RF, LB, RB): ", lFMotor.getCurrent(CurrentUnit.AMPS), rFMotor.getCurrent(CurrentUnit.AMPS), lRMotor.getCurrent(CurrentUnit.AMPS), rRMotor.getCurrent(CurrentUnit.AMPS));
            lclLogger.Debug("Encoder Counts (LF, RF, LB, RB): ", lFMotor.getCurrentPosition(), rFMotor.getCurrentPosition(), lRMotor.getCurrentPosition(), rRMotor.getCurrentPosition());
            lclLogger.Debug("************* TestRobot Set Drive Motor Power TestRobot Set Drive Motor Power **********");
        }

    }


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


    public void setDTMotorTargetPosition(int encoderPosition) {

        lFMotor.setTargetPosition(encoderPosition);
        rFMotor.setTargetPosition(encoderPosition);
        lRMotor.setTargetPosition(encoderPosition);
        rRMotor.setTargetPosition(encoderPosition);

    }


    public void runWithEncoders() {

        lFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    public void runWithEncodersSTP() {

        lRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public void runWithoutEncoders() {

        lFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void resetEncoders() {

        lFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


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
