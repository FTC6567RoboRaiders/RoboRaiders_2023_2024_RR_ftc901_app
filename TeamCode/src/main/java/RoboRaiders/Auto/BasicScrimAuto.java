package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import RoboRaiders.Pipelines.StevesPipeline2;
import RoboRaiders.Robots.NotPirsus;

@Autonomous
public class BasicScrimAuto extends LinearOpMode {

    public NotPirsus robot = new NotPirsus();

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    public StevesPipeline2 pipeline = new StevesPipeline2();

    private boolean isRed = false;
    private boolean droneSide = false;
    private boolean driveInner = false;
    private boolean waitForPartner = false;

    private boolean selectionsAreGood = false;

    double numOfTicks;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        robot.runWithEncoders();

        robot.resetEncoders();
        robot.runWithEncoders();

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("findContoursOutputSizeBlue", pipeline.getFindContoursOutputSize());
            telemetry.addData("filterContoursOutputSizeBlue", pipeline.getFilterContoursOutputSize());
            if(pipeline.contourFound()) {
                telemetry.addData("yes", 1);
            }
            else {
                telemetry.addData("no", 0);
            }
            telemetry.update();

        }

    }

}
