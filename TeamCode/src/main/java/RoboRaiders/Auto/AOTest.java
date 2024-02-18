package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import RoboRaiders.Auto.AutoOptions.AutoOptions;
import RoboRaiders.Pipelines.GripPipelineBlue;
import RoboRaiders.Pipelines.GripPipelineRed;
import RoboRaiders.Robots.Hubbot;


@Autonomous


public class AOTest extends LinearOpMode {

    public Hubbot robot = new Hubbot();
    AutoOptions AO = new AutoOptions(this);

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    GripPipelineRed redPipeline;
    GripPipelineBlue bluePipeline;

    private boolean isRed = false;
    private boolean stageSide = false;
    private boolean waitForPartner = false;

    private boolean selectionsAreGood = false;

    double numOfTicks;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int waitTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        while (!selectionsAreGood) {

            isRed = AO.selectAlliance();              // red or blue
            stageSide = AO.selectStartLocation();         // starting near the drones or the backboard
            waitForPartner = AO.selectWait();                   // wait for partner
            if(waitForPartner){
                waitTime = AO.selectWaitTime();
            }
            // Add new/additional auto options, so things like drive to depot, drop team marker, etc..


            // Display the options selected
            // We show two options per line, to save space and lines.  The maximum number of characters
            // per line is roughly 45.  Maximum number of lines to be displayed is 9.
            // Note: To keep the autonomous options displayed, the automagical clearing of the telemetry data will be
            //       turned off with the setAutoClear(false) prior to calling selectionsGood().  After selectionsGood()
            //       turn on the automagical clearing of the telemetry data which is the default action.
            telemetry.setAutoClear(false);
            telemetry.addLine().addData("Autonomous", "Selections");
            telemetry.addLine().addData("Alliance:", isRed ? "Red  " : "Blue  ").addData("  Robot Start Location:", stageSide ? "Stage" : "Back Stage");
            telemetry.addLine().addData("Wait for Partner:", waitForPartner ? "Yes" : "No");
            if(waitForPartner) {
                telemetry.addLine().addData("Wait Time: ", waitTime);
            }
            telemetry.update();

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, we'll ask again.

            selectionsAreGood = AO.selectionsGood();
            telemetry.setAutoClear(true);
            telemetry.update();    // Clear the selections
        }

        telemetry.addLine().addData("Waiting your command", true);
        telemetry.update();
        waitForStart();

        telemetry.addLine().addData("Selected alliance:", isRed);
        telemetry.addLine().addData("Selected side:", stageSide);
        telemetry.addLine().addData("Wait:", waitForPartner);
        if(waitForPartner){
            telemetry.addLine().addData("Wait Time: ", waitTime);

        }

        /** power controls:
         *  -, -, -, - | forward
         *  +, +, +, + | backward
         *  -, +, +, - | left
         *  +, -, -, + | right
         */

    }

}
