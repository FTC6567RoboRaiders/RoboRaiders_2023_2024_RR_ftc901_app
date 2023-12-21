package RoboRaiders.Utilities.RobotTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotTelemetryDisplay {

    /**
     * RobotTelemetryDisplay displays telemetry data of the robot.  This class assumes 9 lines per
     * display and 45 characters per line.
     *
     * Format for telemetry display:
     *
     * Line 1:    Title Line
     * Line 2:    Robot Status
     * Line 3-8:  Variable Section
     *
     * Example:
     * Line 1:    Nostromo: Sampling
     * Line 2:    Drive Motor Power: 0.45
     * Line 3:    Heading: 95
     * Line 4:    Intake Arm: Extended
     * Line 5:    Encoder Value: 6784
     * Line 6:    Lift/Hang: Extended
     * Line 7:    Lift Claw: Closed
     *
     */

    OpMode op;
    String robotName;
    int displayLineCount = 0;
    String robotStatus;



    /**
     * Constructor for RobotTelemetryDisplay
     * @param op the OpMode tied to this class
     * @param robotName name of the robot
     */
    public RobotTelemetryDisplay(OpMode op, String robotName) {
        this.op = op;
        this.robotName = robotName;
    }

    /**
     * will display the robot telemetry data as shown above
     * @param robotStatus - current robot status (e.g. Deployed, Sampling, etc.)
     *
     */
    public void displayRobotTelemetry(String robotStatus) {

        displayLineCount = 0;
        this.robotStatus = robotStatus;

        // Tell update to clear the display and then update to clear the display
        op.telemetry.setAutoClear(true);
        op.telemetry.update();

        // Tell update to not clear the display when update() is called
        op.telemetry.setAutoClear(false);

        // Output the robot status
        op.telemetry.addLine().addData(robotName,robotStatus);
        op.telemetry.update();

        displayLineCount++;

    }

    /**
     * will display the robot telemetry data as shown above
     * @param robotStatus - current robot status (e.g. Deployed, Sampling, etc.)
     * @param varInfo - string array of variable information that would be helpful to display, only the
     *                  first 6 elements will be displayed
     */

    /**
     * will display the tobot telemetry data as shown above
     * @param label - label of the information to be displayed
     * @param info - the information to be displayed
     */
    public void displayRobotTelemetry(String label, String info) {

        displayLineCount++;

        if (displayLineCount > 7) {
            displayRobotTelemetry(robotStatus);
        }

        // Output the robot status
        op.telemetry.addLine().addData(label,info);
        op.telemetry.update();


    }

}