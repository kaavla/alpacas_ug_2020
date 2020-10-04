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

        debugPath.computeCurvatureForAllPoints();

        debugPath.ToString();
    }

}
