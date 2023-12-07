package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * <b><u>AutoOptions</u></b> class is used to isolate the autonomous options mechanism from the main autonomous classes.
 * It handles displaying the choices and processing the choice made by the driver/coach and returning a boolean
 * (TRUE or FALSE) that indicates which choice was made.  This class is designed around two choices per option
 * but can easily be enhanced to use more than 2 choices.
 *
 * gamepad1 X and B buttons are used to make the selections, thus the need for the opMode to be passed in
 * when the class is instantiated.
 *
 * When using two choices a boolean value (TRUE or FALSE) is return indicating the choice made.
 *
 * Notes:
 *
 * The constructure for this class requires the opMode to be passed in, this allows the method to
 * utilize gamepad1 and telemetry.
 *
 * When adding methods, follow the pattern set by the current methods of setting up a selection array
 * of two, then call the makeSelection method, passing in the prompt and the selection array.
 * Finally, compare the returned integer to 0 and return the boolean value of the compare.
 *
 * Specific Methods
 * <ol>
 * <li>selectAlliance - allows the driver/coach to select the alliance (Red or Blue) </li>
 * <li>selectLocation - allows the driver/coach to select the location the robot is starting at (Depot or Crater)</li>
 * <li>selectDeployFromLander - allows the driver/coach to select if the robot is to be deployed or not from  (Yes or No)</li>
 * <li>selectionsGood - allows the driver/coach to verify that the selections made are good (Yes or No)</li>
 * </ol>
 *
 * Generalized Methods (should never need to be changed)
 * <ol>
 * <li>makeYesNoSelection - generalized method to handle those autonomous selections that are Yes/No</li>
 * <li>makeSelection - generalized method to handle the selection of</li>
 * </li>
 * </ol>
 *
 *
 */

public class AutoOptions {

    private LinearOpMode op;

    private boolean prev_B_ButtonState;                                // "b" button previous state
    private boolean prev_X_ButtonState;                                // "x" button previous state

    /**
     * Constructor
     *
     * @param op - the linear opmode tied to this class
     */
    public AutoOptions(LinearOpMode op) {
        this.op = op;
    }

    /**
     * will return the alliance selection
     *
     * @return a boolean indicating the alliance selection, when:
     * RED  - true
     * BLUE - false
     */
    public boolean selectAlliance() {

        // create paths
        String[] alliances = new String[]{"Red", "Blue"};

        // let driver make selection: index = 0 means first selection, index = 1 means second selection
        int index = makeSelection("Alliance?", alliances);

        // check index against 0: 0 = true, else = false
        return index == 0;
    }

    /**
     * will return the location selection
     *
     * @return a boolean indicating the alliance selection, when:
     * DEPOT  - true
     * CRATER - false
     */
    public boolean selectStartLocation() {

        // create paths
        String[] locations = new String[]{"Drone", "Stage"};

        // let driver make selection: index = 0 means first selection, index = 1 means second selection
        int index = makeSelection("Robot Start Location", locations);

        // check index against 0: 0 = true, else = false
        return index == 0;
    }

    public boolean selectDrivePath() {

        // create paths
        String[] paths = new String[]{"Inner", "Outer"};

        // let driver make selection: index = 0 means first selection, index = 1 means second selection
        int index = makeSelection("robot drive path", paths);

        // check index against 0: 0 = true, else = false
        return index == 0;
    }

    /**
     * This will return yes or no if you want to wait.
     */

    public boolean selectWait() {

        // Let the driver make a yes or no selection to wait for the alliance partner
        int index = makeYesNoSelection("Wait for partner?");

        // check index against 0: 0 = true, else = false
        return index == 0;
    }

    /**
     * will return yes or no if the autonomous selections are good
     * @return a boolean indicating if the autonomous selections are good
     *         YES - true
     *         NO  - false
     */
    public boolean selectionsGood() {

        // let the driver confirm autonomous settings
        int index = makeYesNoSelection("Are selections good");

        // check index against 0: 0 = true, else = false
        return index == 0;

    }

    /**
     * process a yes/no prompt
     * @param msYNPrompt - the prompt for the Yes/No question
     * @return index of the selection, when
     *         Yes - index = 0
     *         No  - index = 1
     */
    public int makeYesNoSelection(String msYNPrompt) {

        String[] yesNo = new String[] { "Yes", "No"};
        int index = makeSelection(msYNPrompt,yesNo);
        return index;
    }

    /**
     * will save the response (selOptions) from a set of 2 possible responses (posResps)
     * a given prompt (selPrompt)
     *
     * @param msPrompt The given configuration prompt
     * @param msResps The possible responses to a given configuration prompt
     *
     *
     */

    private int makeSelection(String msPrompt, String[] msResps) {

        int index = 0;

        // let the user select, reset gamepad1
        op.gamepad1.reset();

        // assume that neither the B nor X button has been pressed
        prev_B_ButtonState = false;
        prev_X_ButtonState = false;


        // prompt user for selection
        op.telemetry.addLine(msPrompt);
        op.telemetry.addLine(msResps[0] + " - X " + " or " + msResps[1] + " - B" );
        op.telemetry.update();

        // loop until either the "b" button or the "x" button is pressed, initially we assume that
        // both buttons have not been pressed.
        //
        // The logic here says OR the previous button states and when they are both false continue
        // here is a table of how this works
        //    +--------------------+------+--------------------+--------+-------------+
        //    | prev_B_ButtonState | -OR- | prev_X_ButtonState | Result | Neg. Result |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      FALSE         | -OR- |     FALSE          | FALSE  |   TRUE      |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      FALSE         | -OR- |     TRUE           | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      TRUE          | -OR- |     FALSE          | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      TRUE          | -OR- |     TRUE           | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+

        while (!(prev_X_ButtonState | prev_B_ButtonState)) {

            // When the X button has been pushed AND the X button was not pushed before
            if (op.gamepad1.x && !prev_X_ButtonState) {
                index = 0;                                            // first response was selected, store the response
                prev_X_ButtonState = true;                            // indicate that the X button state has been PUSHED
            }

            // When the B button has been pushed AND the B button was not pushed before
            else if (op.gamepad1.b && !prev_B_ButtonState) {
                index = 1;                                            // second response was selected, store the response
                prev_B_ButtonState = true;                            // indicate that the B button state has been PUSHED
            }
        }

        // Wait one second
        try {Thread.sleep(750);} catch (InterruptedException e) {e.printStackTrace();}

        return index;
    }

}