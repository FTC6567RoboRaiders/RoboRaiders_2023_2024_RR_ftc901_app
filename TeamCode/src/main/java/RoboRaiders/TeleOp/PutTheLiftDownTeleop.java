package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robots.Pirsus;



@TeleOp(name = "Put The Lift Down Teleop",group="Teleop")
@Disabled


/**
 * Created by Steeeve Kocik for RoboRaiders Testing
 *
 * Change Id      Person          Date          Comments
 * SMK1           Steeeve Kocik   231130        Initial version
 */
public class PutTheLiftDownTeleop extends OpMode {

    public Pirsus robot = new Pirsus();

    @Override
    public void init() {

        // initialise robot and tell user that the robot is initialized
        robot.initialize(hardwareMap);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();

    }

    @Override
    public void loop() {

        double leftStickY = gamepad2.left_stick_y;

        if(leftStickY > 0.0) {
            robot.liftUp();
        }
        else if(leftStickY < 0.0) {
            robot.liftDown();
        }
        else {
            robot.liftStop();
        }

    }

}