package RoboRaiders.Tests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.Hubbot;

@TeleOp(name = "Hubbot Teleop",group="Test")

public class HubTeleop extends LinearOpMode {

    public Hubbot robot = new Hubbot();
    double pwm  = 0.2004;
   double incrementor = 0.0004;





    @Override
    public void runOpMode() {


        // initialise robot and tell user that the robot is initialized
        robot.initialize(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            robot.setControllerPosition(pwm);
            sleep(1000);                                             // Let's wait 1/2 a second, why? because I said so
            telemetry.addData("RR Blinkin Class Test", "");
            telemetry.addData("Press GamePad1.A to Increment to the next Blinkin Setting", "");
            telemetry.addData("Press GamePad1.B to Decrement to the previous Blinkin Setting", "");
            telemetry.addData("----------------------------------------------------------", "");
            telemetry.addData("RR Blinkin Setting: ", "%5.5f", robot.getControllerPosition());
            telemetry.addData("RR Blinkin Ordinal: ", robot.getOrdinal(pwm));
            telemetry.addData("Pattern :", robot.getPattern(robot.getControllerPosition()));
            telemetry.addData(">", "Press Stop to End Test.");
            telemetry.update();

            if ((pwm + incrementor) > 1.0) pwm = 1.0;
            else pwm += incrementor;


            // When gp1.a is pushed, advance/increment the servo position by incrementor
            // Safety Tip: care must be taken such that the motor power will not advance
            //             beyond 1.0 (the lower limit), so check for such instances and
            //             don't allow the servo to advance beyond 1.0.
//            if (gamepad1.a) {
//                if ((pwm + incrementor) > 1.0) pwm = 1.0;
//                else pwm += incrementor;
//            }
//            if (gamepad1.b) {
//                if ((pwm - incrementor) < 0.2004) pwm = 0.2004;
//                else pwm -= incrementor;
//            }


        }
    }
}
