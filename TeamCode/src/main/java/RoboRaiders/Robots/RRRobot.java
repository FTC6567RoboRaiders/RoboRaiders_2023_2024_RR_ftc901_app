package RoboRaiders.Robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RRRobot {

    public int xCoord;
    public int yCoord;
    public int xBCoord;
    public int yBCoord;

    public void initialize(HardwareMap ahwMap){
    }
    public void setX(int xVal){xCoord = xVal;}
    public void setY(int yVal){
        yCoord = yVal;
    }
    public int getBX(){
        return xBCoord;
    }
    public int getBY(){
        return yBCoord;
    }
    public void setBX(int xBVal){
        xBCoord = xBVal;
    }
    public void setBY(int yBVal){
        yBCoord = yBVal;
    }

}
