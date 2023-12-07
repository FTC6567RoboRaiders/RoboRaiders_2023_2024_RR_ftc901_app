package RoboRaiders.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.Pirsus;

@TeleOp(name = "Bucket Servo Incrementor",group="Servo")

/**
 * Created by Steeeve Kocik for RoboRaiders Testing
 *
 * Change Id      Person          Date          Comments
 * SMK1           Steeeve Kocik   231130        Initial version
 */
public class BucketServoTest extends LinearOpMode {

 public Pirsus robot = new Pirsus();

    public double servoPosition = 0.59;
    public double incrementor = 0.05;



    @Override
    public void runOpMode() {

        robot.adjustBucketPosition(servoPosition);

        waitForStart();

        while (opModeIsActive()) {

            robot.adjustBucketPosition(servoPosition);
            sleep(500);                                             // Let's wait 1/2 a second, why? because I said so
            telemetry.addData("Bucket Servo Position Test", "");

            telemetry.addData("----------------------------------------------------------","");
            telemetry.addData("Press GamePad1.A to Increase The Servo Position", "");
            telemetry.addData("Press GamePad1.B to Decrease The Servo Position", "");
            telemetry.addData("Press GamePad1.X to Increase Amount To Increment/Decrement","");
            telemetry.addData("Press GamePad1.Y to Decrease Amount to Increment/Decrement","");
            telemetry.addData("----------------------------------------------------------","");
            telemetry.addData("Servo Position: ", "%5.2f", servoPosition);
            telemetry.addData("Servo Position Increments/Decrements by: ","%5.2f", incrementor);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // When gp1.a is pushed, advance/increment the servo position by incrementor
            // Safety Tip: care must be taken such that the servo position will not advance
            //             beyond 1.0 (the lower limit), so check for such instances and
            //             don't allow the servo to advance beyond 1.0.
            if (gamepad1.a) {
                if (servoPosition + incrementor > 1.0) servoPosition = 1.0;
                else servoPosition += incrementor;
            }

            // When gp1.b is pushed, retract/decrement the servo position by incrementor
            // Safety Tip: care must be taken such that the servo position will not advance
            //             beyond 0.0 (the lower limit), so check for such instances and
            //             don't allow the servo to advance beyond 0.0.
            if (gamepad1.b) {
                if (servoPosition + incrementor < 0.0) servoPosition = 0.0;
                else servoPosition -= incrementor;
            }

            // When gp1.x is pushed, increase the increment rate by 0.05
            // Safety Tip: only allow a maximum increment of 1.0
            if (gamepad1.x) {
                incrementor += 0.05;
                if (incrementor > 1.0) incrementor = 1.0;
            }

            // When gp1.y is pushed, decrease the increment rate by 0.05
            // Safety Tip: only allow a minimum increment of -1.0

            if (gamepad1.y) {
                incrementor -= 0.05;
                if (incrementor < -1.0) incrementor = -1.0;
            }

            robot.adjustBucketPosition(servoPosition);
        }

        sleep(5000);
    }

}