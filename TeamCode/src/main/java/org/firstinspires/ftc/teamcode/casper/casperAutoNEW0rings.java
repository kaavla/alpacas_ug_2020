package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_UP;

@Autonomous(group = "robot")

public class casperAutoNEW0rings extends casperAutonomousBase {
    @Override

    public void runOpMode() throws InterruptedException {
        robot = new casperMecanumDrive(hardwareMap);

        robot.initVuforia(hardwareMap);
        robot.initTfod(hardwareMap);
//start position
        Pose2d startPose = new Pose2d(-63, -57, Math.toRadians(0));
        robot.setPoseEstimate(startPose);
        //origin at middle of full field(0,0)
        //starting position down red (-60, -48, 90)

        //rings red located at (-24, -36)
        //position for shooting at (-12, -51)
        // position 4 square at (58, -58)

        waitForStart();
//going forward to detect rings
        Trajectory traj0 = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-24, -57), Math.toRadians(0))
                .build();
        //int position = 4;
        int pos = getNumRings(1500); //ms
        telemetry.addData(">", "Num of rings = %d", pos);
        telemetry.update();

        if (pos == 0) {
            telemetry.addData(">", "running 0 ring path");
            telemetry.update();

//going to drop wobble goal
            Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    .splineTo(new Vector2d(0, -50), Math.toRadians(45))
                    .build();
            robot.followTrajectory(traj1);

//dropping wobble goal
            robot.shootMotorLeft.setPower(0.8);
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            sleep(400);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(400);

//going to shooting position + shooting
            Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    .splineTo(new Vector2d(-12, -51), Math.toRadians(170))
                    .build();
            robot.followTrajectory(traj2);
            robot.autonomousShoot();
            sleep(5000);
            robot.stopAllMotors();

//replace above with powershot stuff



//picking up second wobble goal
            Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                    .lineToSplineHeading(new Pose2d(-25, -24, Math.toRadians(270)))
                    .build();
            robot.followTrajectory(traj3);

            moveWobbleGoal(WOBBLE_GOAL_DOWN);

            Trajectory traj4 = robot.trajectoryBuilder(traj3.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    .strafeRight(9)
                    .build();
            robot.followTrajectory(traj4);
            sleep(100);
            robot.closeWobbleClaw();
            sleep(1000);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(500);
//dropping second wobble goal, going back to same position in traj1
            Trajectory traj5 = robot.trajectoryBuilder(traj4.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    .splineTo(new Vector2d(0, -50), Math.toRadians(45))
                    .build();
            robot.followTrajectory(traj5);
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            sleep(200);
            robot.openWobbleClaw();
            sleep(400);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(300);

//park on white line
            Trajectory traj6 = robot.trajectoryBuilder(traj5.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                    .lineToSplineHeading(new Pose2d(12, -40, Math.toRadians(90)))
                    .build();
            robot.followTrajectory(traj6);
        }


    }

}