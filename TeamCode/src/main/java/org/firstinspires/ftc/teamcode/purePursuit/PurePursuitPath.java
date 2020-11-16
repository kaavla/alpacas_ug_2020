package org.firstinspires.ftc.teamcode.purePursuit;

import android.content.Context;
import android.graphics.Canvas;
import android.util.AttributeSet;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitPath {

    //Creates a list for the user defined points
    List<Waypoint> Points = new ArrayList<Waypoint>();
    //UPath stands for Updated Path, a list for the new and user defined points
    List<Waypoint> UPath = new ArrayList<Waypoint>();
    //SPath stands for Smoothed Path, a list for the points that have been smoothed out
    List<Waypoint> SPath = new ArrayList<Waypoint>();

    // Creates a list of the different arrays which can be chosen as parameters
    public enum ArrayLS {
        Points, UPath, SPath
    }

    //This is a function that adds user defined points to the list Points
    public void addUserWaypoint(double x, double y) {
        //creates a waypoint P1
        Waypoint P1 = new Waypoint(x, y, true);
        Points.add(P1);
    }

    //This function adds Waypoints to the UPath list, either user defined or not
    public void addProcessedWaypoint(double x, double y, boolean userInserted) {
        //creates a waypoint P1
        Waypoint P1 = new Waypoint(x, y, userInserted);
        UPath.add(P1);
    }

    public void addSmoothingWaypoint(double x, double y, boolean userInserted) {
        //creates a waypoint P1
        Waypoint P1 = new Waypoint(x, y, userInserted);
        SPath.add(P1);
    }

    public void ToString(ArrayLS array) {
        if(array == ArrayLS.Points) {
            for (int i = 0; i < Points.size(); i++) {
                Waypoint Point = Points.get(i);
                System.out.println(ConsoleColors.YELLOW + (i + 1) + ": " + Point.ToString() + ConsoleColors.RESET);
            }
        }

        if(array == ArrayLS.UPath) {
            for (int i = 0; i < UPath.size(); i++) {
                Waypoint Point = UPath.get(i);
                System.out.println(ConsoleColors.PURPLE + (i + 1) + ": " + Point.ToString() + ConsoleColors.RESET);
            }
        }

        if(array == ArrayLS.SPath) {
            for (int i = 0; i < SPath.size(); i++) {
                Waypoint Point = SPath.get(i);
                System.out.println(ConsoleColors.GREEN + (i + 1) + ": " + Point.ToString() + ConsoleColors.RESET);
            }
        }
    }

    //This calculates the curvature at any point
    public double computeCurvatureForPoint(Waypoint Current, Waypoint Prev, Waypoint Next) {
        //sets all the variables
        double curvature;
        double radius;
        double k1;
        double k2;
        double circleCenterX;
        double circleCenterY;
        double x1 = Current.xCoord;
        double y1 = Current.yCoord;
        double x2 = Prev.xCoord;
        double y2 = Prev.yCoord;
        double x3 = Next.xCoord;
        double y3 = Next.yCoord;

        if (x1 == x2) {
            // If x1 and x2 are equal, we will have a division by 0 error so we add very little
            // to one number, keeping the calculation very accurate
            x1 = x1 + 0.0001;
        }

        //Computations
        k1 = (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2)) / (2 * (x1 - x2));
        k2 = (y1 - y2) / (x1 - x2);
        circleCenterY = (Math.pow(x3, 2) + Math.pow(y3, 2) - Math.pow(x2, 2) - Math.pow(y2, 2) -
                (2 * k1 * (x3 - x2))) / (2 * (y3 - y2 - (k2 * x3) + (k2 * x2)));
        circleCenterX = k1 - (circleCenterY * k2);

        radius = Math.sqrt(Math.pow(x1 - circleCenterX, 2) + Math.pow(y1 - circleCenterY, 2));
        curvature = 1 / radius;

        // System.out.println("radius = " + radius);

        //enters the value of curvature and sets the double "computeCurvatureForPoint" equal to that value
        return curvature;
    }

    //This function applies the Curvature function for all points in the array
    public void computeCurvatureForAllPoints(ArrayLS arrayChoice) {
        Waypoint Previous;
        Waypoint Current;
        Waypoint Next;

        if (arrayChoice == ArrayLS.UPath) {
        //the for loop applies the same steps to every point in the UPath list
            for (int i = 0; i < UPath.size(); i++) {
                //for the first and last point, there is no point before, or after, meaning that the
                //curvature will be = 0. We make the curvature of these points = 0 manually.
                if (i == 0 || i == UPath.size() - 1) {
                    UPath.get(i).curvature = 0;
                    continue;
                }
                Previous = UPath.get(i - 1);
                Current = UPath.get(i);
                Next = UPath.get(i + 1);
                //uses the curvature function to find the curvature of all of the points
                UPath.get(i).curvature = computeCurvatureForPoint(Current, Previous, Next);
                double curvature = UPath.get(i).curvature;
                System.out.println((i+1) + ": radius = " + (1/curvature));
            }
        }

        if (arrayChoice == ArrayLS.SPath) {
            //the for loop applies the same steps to every point in the UPath list
            for (int i = 0; i < SPath.size(); i++) {
                //for the first and last point, there is no point before, or after, meaning that the
                //curvature will be = 0. We make the curvature of these points = 0 manually.
                if (i == 0 || i == SPath.size() - 1) {
                    SPath.get(i).curvature = 0;
                    continue;
                }
                Previous = SPath.get(i - 1);
                Current = SPath.get(i);
                Next = SPath.get(i + 1);
                //uses the curvature function to find the curvature of all of the points
                SPath.get(i).curvature = computeCurvatureForPoint(Current, Previous, Next);
                double curvature = SPath.get(i).curvature;
                System.out.println((i+1) + ": radius = " + (1/curvature));
            }
        }
    }

    // The distance formula
    public double distanceFormula(double x1, double y1, double x2, double y2) {
        double dist;
        dist = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        return dist;
    }

    //This function adds waypoints to the UPath list and also adds the pre-defined waypoints to it.
    void pointInsertion(double spacing) {
        Waypoint Current;
        Waypoint Next;
        int numPointsThatFit;

        //for every point within the below requirements, the current point is added, and
        //every point that can fit between with 6 inch spacing is also added to the UPath list.
        for (int i = 0; i < Points.size(); i++) {
            Current = Points.get(i);

            //add the last user defined point to the new list
            if (i == Points.size() - 1) {
                addProcessedWaypoint(Current.xCoord, Current.yCoord, Current.userDefined);
                break;
            }
            Next = Points.get(i + 1);

            //This adds the current point to the list
            addProcessedWaypoint(Current.xCoord, Current.yCoord, Current.userDefined);

            //This finds out how many points can fit between the two points with the needed spacing
            numPointsThatFit = (int) Math.floor(distanceFormula(Current.xCoord, Current.yCoord,
                    Next.xCoord, Next.yCoord) / spacing);

            //This finds the change in x and change in y to get from point A to B
            double ChangeInXStart = Next.xCoord - Current.xCoord;
            double ChangeInYStart = Next.yCoord - Current.yCoord;

            //This finds the change in X and Change in Y in order to have the correct spacing
            //between points
            double ChangeInXSpacing = ChangeInXStart / numPointsThatFit;
            double ChangeInYSpacing = ChangeInYStart / numPointsThatFit;

            //This adds the new points to the list
            for (int index = 1; index < numPointsThatFit; index++) {
                double newX = Current.xCoord + ChangeInXSpacing * index;
                double newY = Current.yCoord + ChangeInYSpacing * index;
                addProcessedWaypoint(newX, newY, false);
            }
        }
    }

    void pathSmoothing(double weight_data, double weight_smooth, double tolerance) {
        //Todo find out why this works
        double a = weight_data;
        double b = weight_smooth;
        double changeX = tolerance;
        double changeY = tolerance;
        a = 1-b;

        // Copies the UPath Array to the SPath Array
        for(int i = 0; i < UPath.size(); i++) {
            addSmoothingWaypoint(UPath.get(i).xCoord, UPath.get(i).yCoord, false);
        }

        // The part that smooths the path
        while(changeX >= tolerance && changeY >= tolerance) {
            changeX = 0;
            changeY = 0;

            for(int i = 1; i < SPath.size() - 1; i++) {
                // Looks at 3 consecutive points: Prev(Previous), Temp(Temporary), and Next
                // The value of the temporary placeholder is stored in auxX and auxY to be compared
                //      to the now changed TempX and TempY
                double PrevX = SPath.get(i-1).xCoord;
                double PrevY = SPath.get(i-1).yCoord;
                double TempX = SPath.get(i).xCoord;
                double TempY = SPath.get(i).yCoord;
                double NextX = SPath.get(i+1).xCoord;
                double NextY = SPath.get(i+1).yCoord;
                double auxX = TempX;
                double auxY = TempY;

                // I have to change the X and Y separately from each other
                // x coordinate side of the equation
                TempX += a * (UPath.get(i).xCoord - TempX) + b * (PrevX + NextX - (2 * TempX));

                // y coordinate side of the equation
                TempY += a * (UPath.get(i).yCoord - TempY) + b * (PrevY + NextY - (2 * TempY));

                // Replaces the corresponding waypoint with the new X and Y values that were created
                Waypoint P1 = new Waypoint(TempX, TempY, false);
                SPath.set(i, P1);

                // Calculates how much the point was changed and uses it to see if the point needs
                //      to be changed more.
                changeX += Math.abs(auxX - TempX);
                changeY += Math.abs(auxY - TempY);

            }
        }
    }


    // Calculates the target velocity at a defined point
    public void targetVelocity(ArrayLS arrayChoice, double k) {
        // Sets the max velocity for the path(the value is just a placeholder).
        final double pathMaxVel = 10;

        // Sets max acceleration for the path(the value is just a placeholder).
        //      Calling it "a" to keep the equations short
        final double a = 4;

        int lastP = SPath.size();
        SPath.get(lastP).velocity = 0;
        for(int i = (SPath.size() - 1); i > 0; i--) {
            // defines the waypoint name
            Waypoint Point = SPath.get(i);
            Waypoint APoint = SPath.get(i+1);

            // Takes PathMaxVal and k/curvature and outputs the smallest of the two as the max velocity
            //      for the point.
            Point.maxVel = Math.min(pathMaxVel, k / Point.curvature);

            // Calculates the distance between the two points.
            double Dist;
            Dist = distanceFormula(APoint.xCoord, APoint.yCoord, Point.xCoord, Point.yCoord);

            // Sets the old velocity at the point(Not quite sure about this or what it does so I'm
            //      setting a placeholder here for now).
            double oldTargVel = 3;

            // Sets the target velocity.
            Point.velocity = Math.min(oldTargVel, Math.sqrt(Math.pow(APoint.velocity, 2) + 2 * a * Dist));

            // Ensures that the target velocity is always <= the max velocity set for the point.
            if(Point.velocity > Point.maxVel) {
                Point.velocity = Point.maxVel;
            }

        }
    }
}

