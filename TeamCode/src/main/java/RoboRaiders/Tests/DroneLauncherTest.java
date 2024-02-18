package RoboRaiders.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import RoboRaiders.Robots.Pirsus;

@TeleOp(name = "Drone Launcher Test",group="Test")
@Disabled

/**
 * Created by Steeeve Kocik for RoboRaiders Testing
 *
 * Change Id      Person          Date          Comments
 * SMK1           Steeeve Kocik   231130        Initial version
 */
public class DroneLauncherTest extends LinearOpMode {

    public DcMotorEx droneMotor = null;
    public double motorPower = 0.5;
    public double incrementor = 0.05;



    @Override
    public void runOpMode() {


        droneMotor  = hardwareMap.get(DcMotorEx.class, "droneMotor");

        waitForStart();

        while (opModeIsActive()) {


            sleep(500);                                             // Let's wait 1/2 a second, why? because I said so
            telemetry.addData("Drone Launcher Test", "");

            telemetry.addData("----------------------------------------------------------","");
            telemetry.addData("**** Press GamePad1.LeftBumper to Launch Drone ****", "");
            telemetry.addData("==========================================================","");
            telemetry.addData("Press GamePad1.A to Increase the Motor Power", "");
            telemetry.addData("Press GamePad1.B to Decrease the Motor Power", "");
            telemetry.addData("Press GamePad1.X to Increase Amount to Increment/Decrement","");
            telemetry.addData("Press GamePad1.Y to Decrease Amount to Increment/Decrement","");
            telemetry.addData("----------------------------------------------------------","");
            telemetry.addData("Drone Motor Power: ", "%5.2f", motorPower);
            telemetry.addData("Motor Power Increments/Decrements by: ","%5.2f", incrementor);
            telemetry.addData(">", "Press Stop to End Test." );
            telemetry.update();

            // When gp1.a is pushed, advance/increment the servo position by incrementor
            // Safety Tip: care must be taken such that the motor power will not advance
            //             beyond 1.0 (the lower limit), so check for such instances and
            //             don't allow the servo to advance beyond 1.0.
            if (gamepad1.a) {
                if (motorPower + incrementor > 1.0) motorPower = 1.0;
                else motorPower += incrementor;
            }

            // When gp1.b is pushed, retract/decrement the servo position by incrementor
            // Safety Tip: care must be taken such that the motor power will not advance
            //             beyond 0.0 (the lower limit), so check for such instances and
            //             don't allow the servo to advance beyond 0.0.
            if (gamepad1.b) {
                if (motorPower + incrementor < 0.0) motorPower = 0.0;
                else motorPower -= incrementor;
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

            // When the gamepad1.left_bumper is pushed, apply power to the drone motor
            if (gamepad1.left_bumper)  droneMotor.setPower(-motorPower);
            else droneMotor.setPower(0.0);
        }

        sleep(5000);
    }

}