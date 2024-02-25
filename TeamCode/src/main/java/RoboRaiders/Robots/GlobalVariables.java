package RoboRaiders.Robots;

public class GlobalVariables {

    private static double autoHeading = 0.0;
    private static boolean allianceColour = false; // true - red | false - blue
    private static boolean startSide = false; // true - stage | false - backstage

    public static void setAutoHeading(double heading) {
        autoHeading = heading;
    }

    public static double getAutoHeading(){
        return autoHeading;
    }

    public static void setAllianceColour(boolean colour) {
        allianceColour = colour;
    }

    public static boolean getAllianceColour() {
        return allianceColour;
    }

    public static void setSide(boolean side) {
        startSide = side;
    }

    public static boolean getSide() {
        return startSide;
    }

}
