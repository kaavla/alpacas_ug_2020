package org.firstinspires.ftc.teamcode.purePursuit;

public class testPurePursuit extends PurePursuitPath{
    public static void main(String[] args) {
        PurePursuitPath debugPath = new PurePursuitPath();

        debugPath.addUserWaypoint(0,0);
        debugPath.addUserWaypoint(6,8);
        debugPath.addUserWaypoint(12, 16);
        debugPath.addUserWaypoint(18, 24);
        debugPath.addUserWaypoint(24, 16);

        debugPath.pointInsertion(5);
        debugPath.pathSmoothing(0.43, 0.57, 0.001);

        System.out.println(ConsoleColors.BLUE + "______________________" + ConsoleColors.RESET);
        System.out.println("Curvature for UPath");
        debugPath.computeCurvatureForAllPoints(ArrayLS.UPath);
        System.out.println(ConsoleColors.BLUE + "______________________" + ConsoleColors.RESET);
        System.out.println(ConsoleColors.PURPLE_BOLD + "UPath List" + ConsoleColors.RESET);
        debugPath.ToString(ArrayLS.UPath);
        System.out.println(ConsoleColors.BLUE + "______________________" + ConsoleColors.RESET);
        System.out.println("Curvature for SPath");
        debugPath.computeCurvatureForAllPoints(ArrayLS.SPath);
        System.out.println(ConsoleColors.BLUE + "______________________" + ConsoleColors.RESET);
        System.out.println(ConsoleColors.GREEN_BOLD + "SPath List" + ConsoleColors.RESET);
        debugPath.ToString(ArrayLS.SPath);
        System.out.println(ConsoleColors.BLUE + "______________________" + ConsoleColors.RESET);
    }

}