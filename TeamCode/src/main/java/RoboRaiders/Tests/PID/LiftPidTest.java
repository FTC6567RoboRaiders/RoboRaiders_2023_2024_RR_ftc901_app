package RoboRaiders.Tests.PID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import RoboRaiders.Robots.LiftBot;
import RoboRaiders.PID.PidUpdReceiver;
import RoboRaiders.PID.RoboRaidersPID;


@Autonomous (name = "Lift PID Test",group="PID")
@Disabled

public class LiftPidTest extends LinearOpMode {

    // PID variables and classes
    private PidUpdReceiver pidUdpReceiver;
    private RoboRaidersPID rrPID;
    private double kP, kI, kD, distance, direction;

    private LiftBot myLiftBot;

    private double motorPower;
    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        myLiftBot = new LiftBot();

        // Create new instance of receiver
        pidUdpReceiver = new PidUpdReceiver();

        //Create new rrPID
        rrPID = new RoboRaidersPID(0,0,0);

        // Start listening
        pidUdpReceiver.beginListening();

        // set the transmission interval to 50 milliseconds
        telemetry.setMsTransmissionInterval(50);


        // Wait for start to be pushed
        waitForStart();

        if (opModeIsActive()) {

            updatePIDCoefficients();

            rrPID.setCoeffecients(kP,kI,kD);

            telemetry.addData("kP",kP);
            telemetry.addData("kI",kI);
            telemetry.addData("kD",kD);
            telemetry.addData("distance",distance);
            telemetry.addData("direction",direction);
            telemetry.update();

            encoderDrivePIDTuner(distance, direction);
        }

        pidUdpReceiver.shutdown();
    }


    public void updatePIDCoefficients() {

        kP = pidUdpReceiver.getP();
        kI = pidUdpReceiver.getI();
        kD = pidUdpReceiver.getD();
        direction = pidUdpReceiver.getDirection();
        distance = pidUdpReceiver.getDegrees();

        //rtd.displayRobotTelemetry("kP",String.valueOf(kP));
    }
    /**
     * Tuner method to tune the PID variables for lift
     * @param targetDistance - the target distance to travel or extend the lift
     * @param direction 0.0 - extend the lift, 1.0 retract the lift
     */
    public void encoderDrivePIDTuner(double targetDistance, double direction) {

        myLiftBot.resetEncoders();
        myLiftBot.runWithEncoders();
        rrPID.initialize();   // re-initialized the pid variables that we care about

        double targetEncoderCount = Math.abs(myLiftBot.liftCalculateCounts(targetDistance));
        double currentEncoderCount = myLiftBot.getAverageEncoderCount();

        if (direction == 1.0) direction = -1.0; // Retract the lift, motor powers are negative
        else                  direction = 1.0;  // Extend the lift, motor powers are positive

        while (opModeIsActive() && (targetEncoderCount > currentEncoderCount) )
        {
            motorPower = rrPID.CalculatePIDPowers(targetEncoderCount, myLiftBot.getAverageEncoderCount());
            myLiftBot.setLiftMotorPower((direction*motorPower), (direction*motorPower));
            currentEncoderCount = myLiftBot.getAverageEncoderCount();
        }


    }



}