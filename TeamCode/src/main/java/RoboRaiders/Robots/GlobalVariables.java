package RoboRaiders.Robots;

public class GlobalVariables {

    private static double autoHeading = 0.0;

    public static void setAutoHeading(double heading){

        autoHeading = heading;

    }
    public static double getAutoHeading(){
        return autoHeading;
    }
}
