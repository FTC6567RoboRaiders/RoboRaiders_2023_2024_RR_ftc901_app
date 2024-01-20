package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import RoboRaiders.Pipelines.StevesPipeline2;
import RoboRaiders.Robots.NotPirsus;
import RoboRaiders.Utilities.Logger.Logger;

@Autonomous
public class BasicScrimAuto extends LinearOpMode {

    public NotPirsus robot = new NotPirsus();

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
//    public StevesPipeline2 pipeline = new StevesPipeline2();

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
        int[] redCoords = new int[2];
        redCoords = robot.getRed();

        waitForStart();
        while(opModeIsActive()){


            telemetry.addLine().addData("Top Left X and Y COORDINATE During Run: ", String.valueOf(robot.getX()) + ", " + String.valueOf(robot.getY()));
            telemetry.addLine().addData("Bottom Right X and Y COORDINATE During Run: ", String.valueOf(robot.getBX()) + ", " + String.valueOf(robot.getBY()));


            telemetry.addLine().addData("NUMBER OF TIMES CALLED: ", String.valueOf(robot.stevesPipeline.i));
            telemetry.addLine().addData("POSITION:", bluePosition());
            telemetry.addLine().addData("POSITION:", (bluePosition()==0) ? "Left": (bluePosition()==1) ? "Center": "Right");
            telemetry.update();


        }


    }
    public int bluePosition(){
        if(robot.getX() >= 0 && robot.getX()<= 375){
            return 1; //Center Position
        }
        else if(robot.getX() >= 425 && robot.getX()<= 625){
            return 2; //Right Position
        }
        else{
            return 0; //Left Position
        }

    }
    public int redPosition(){
        if(robot.getX() >= 95 && robot.getX()<= 275){
            return 1; //Center Position
        }
        else if(robot.getX() >= 410 && robot.getX()<= 650){
            return 2; //Right Position
        }
        else{
            return 0; //Left Position
        }

    }

}
