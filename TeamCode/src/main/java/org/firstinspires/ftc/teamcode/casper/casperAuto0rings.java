package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_UP;


@Autonomous(group = "robot")
public class casperAuto0rings extends casperAutonomousBase {
    @Override

    public void runOpMode() throws InterruptedException {
        robot = new casperMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-56, -48, Math.toRadians(90));
        robot.setPoseEstimate(startPose);
        //origin at middle of full field(0,0)
        //starting position down red (-60, -48, 90)

        //rings red located at (-24, -36)
        //position for shooting at (-12, -51)
        // position 4 square at (58, -58)

        waitForStart();

        //int position = 4;

        if (isStopRequested()) return;



        Trajectory traj1 = robot.trajectoryBuilder(startPose)
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .splineTo(new Vector2d(-36, -55), Math.toRadians(0))
                .splineTo(new Vector2d(0, -50), Math.toRadians(45))
                .build();
        robot.followTrajectory(traj1);

        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        sleep(400);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        robot.shootMotorLeft.setPower(0.8);

        Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .splineTo(new Vector2d(-12, -51), Math.toRadians(170))
                .build();
        robot.followTrajectory(traj2);
        robot.autonomousShoot();
        sleep(5000);
        robot.stopAllMotors();

        Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                .lineToSplineHeading(new Pose2d(-25, -29, Math.toRadians(270)))
                .build();
        robot.followTrajectory(traj3);

        moveWobbleGoal(WOBBLE_GOAL_DOWN);

        Trajectory traj4 = robot.trajectoryBuilder(traj3.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .strafeRight(9)
                .build();
        robot.followTrajectory(traj4);

        robot.closeWobbleClaw();
        sleep(1000);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        sleep(500);
        Trajectory traj5 = robot.trajectoryBuilder(traj4.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .splineTo(new Vector2d(0, -50), Math.toRadians(45))
                .build();
        robot.followTrajectory(traj5);
        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        Trajectory traj6 = robot.trajectoryBuilder(traj5.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                .lineToSplineHeading(new Pose2d(12, -40, Math.toRadians(90)))
                .build();
        robot.followTrajectory(traj6);

        //robot.stopAllMotors();
/*
        if (position == 0) {
            Trajectory traj2 = robot.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-6, -36), Math.toRadians(90))
                    .build();
            robot.followTrajectory(traj2);
        } else if (position == 1) {
            Trajectory traj2 = robot.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(24, -12), Math.toRadians(90))
                    .build();
            robot.followTrajectory(traj2);
        } else if (position == 4) {
            Trajectory traj2 = robot.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(58, -58), Math.toRadians(90))
                    .build();
            robot.followTrajectory(traj2);
        }
        //traj3 in position to shoot rings into the upper target, and then return to get wobble
        Trajectory traj3 = robot.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(9, 22), Math.toRadians(202))
                .build();
        robot.followTrajectory(traj3);


        Trajectory traj4 = robot.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-30, -4), Math.toRadians(-90))
                .build();
        robot.followTrajectory(traj4);
*/
    }

}

