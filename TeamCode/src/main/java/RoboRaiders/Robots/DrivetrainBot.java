package RoboRaiders.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DrivetrainBot {

    public DcMotorEx lFMotor = null;
    public DcMotorEx rFMotor = null;
    public DcMotorEx lRMotor = null;
    public DcMotorEx rRMotor = null;

    public HardwareMap hwMap = null;

    public DrivetrainBot() {}

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        hwMap = ahwMap;

        lFMotor = hwMap.get(DcMotorEx.class, "lFMotor");
        rFMotor = hwMap.get(DcMotorEx.class, "rFMotor");
        lRMotor = hwMap.get(DcMotorEx.class, "lRMotor");
        rRMotor = hwMap.get(DcMotorEx.class, "rRMotor");

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

        /**
         * This method will set the power for the drive motors
         *
         * @param leftFront  power setting for the left front motor
         * @param rightFront power setting for the right front motor
         * @param leftBack   power setting for the left back motor
         * @param rightBack  power setting for the right back motor
         */

    }

    public void setDriveMotorPower(double leftFront, double rightFront, double leftBack, double rightBack) {

        lFMotor.setPower(leftFront);
        rFMotor.setPower(rightFront);
        lRMotor.setPower(leftBack);
        rRMotor.setPower(rightBack);

    }

}
