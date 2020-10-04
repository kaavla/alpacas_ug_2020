package org.firstinspires.ftc.teamcode.purePursuit;

public class Waypoint {
    //we define variables here
    public double xCoord;
    public double yCoord;
    public double curvature;
    public double velocity;
    public boolean userDefined = false;

    //This is the constructor for the class, allowing us to access the parameters in each Waypoint
    public Waypoint (double x, double y, boolean bUser){
        xCoord = x; //This sets the parameter equal to the variables that we created
        yCoord = y;
        userDefined = bUser;
    }

    public Waypoint () {
        //Leave it empty as a normal setting
    }

    public String ToString() {
        String returnString = "(" + xCoord + ", " + yCoord + ", " + curvature + ")";
        return returnString;
    }

}
