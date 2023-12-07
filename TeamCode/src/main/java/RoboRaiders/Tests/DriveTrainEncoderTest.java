package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.Robots.Pirsus;

@Autonomous (name="Encoder Test - Straight", group="Drive Train")


/**
 * Created by Steeeve Kocik for RoboRaiders Testing
 *
 * Change Id      Person          Date          Comments
 * SMK1           Steeeve Kocik   231130        Initial version
 */
public class DriveTrainEncoderTest extends LinearOpMode {

    public Pirsus robot = new Pirsus();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize and set up the robot's drive motors
        robot.initialize(hardwareMap);             // Initialize the robot
        robot.resetEncoders();                     // Reset the encoder counts
        robot.runWithEncoders();                   // Tell the motors to run with encoders

        telemetry.addData("Status ", "Initialized");
        telemetry.update();

        // Wait for the start button to be pushed
        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Status ", "opModeIsActive");


            robot.setDriveMotorPower(0.5, 0.50, 0.50, 0.50);   // Set power to 50%
            telemetry.addData("Left Rear Drive Encoder Counts", "(%.0f)",robot.getBackLeftDriveEncoderCounts());
            telemetry.addData("Left Front Drive Encoder Counts", "(%.0f)",robot.getFrontLeftDriveEncoderCounts());
            telemetry.addData("Right Rear Drive Encoder Counts", "(%.0f)", robot.getBackRightDriveEncoderCounts());
            telemetry.addData("Right Front Drive Encoder Counts", "(%.0f)", robot.getFrontRightDriveEncoderCounts());


            telemetry.update();
        }


        robot.setDriveMotorPower(0.0,0.0, 0.0, 0.0);         // Motors stop

    }
}
