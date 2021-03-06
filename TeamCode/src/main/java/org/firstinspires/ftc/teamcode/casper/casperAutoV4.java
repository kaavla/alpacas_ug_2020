package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_UP;


@Autonomous(group = "robot")

public class casperAutoV4 extends casperAutonomousBase {
    @Override

    public void runOpMode() throws InterruptedException {
        robot = new casperMecanumDrive(hardwareMap);

        robot.initVuforia(hardwareMap);
        robot.initTfod(hardwareMap);

//start position
        Pose2d startPose = new Pose2d(-63, -57, Math.toRadians(0));
        robot.setPoseEstimate(startPose);
        //origin at middle of full field(0,0)
        // Rings red located at (-24, -36)
        //position for shooting at (-12, -51)
        // position 4 square at (58, -58)

        waitForStart();

        //going forward to detect rings
        Trajectory traj0 = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25, -57), Math.toRadians(0))
                .build();
        robot.followTrajectory(traj0);

        //int position = 4;
        int pos = getNumRings(1500); //ms`
        telemetry.addData(">", "Num of rings = %d", pos);
        telemetry.update();

        if (pos == 4) {
            telemetry.addData(">", "Running 4 ring path");
            telemetry.update();


            Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
//going to start line
                    .splineTo(new Vector2d(12, -50), Math.toRadians(0))
                    .build();
            robot.followTrajectory(traj1);

            //going to wobble goal
            Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                    .splineTo(new Vector2d(50, -50), Math.toRadians(0))
                    .build();
//dropping wobble goal
            robot.shootMotorLeft.setPower(0.8);
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            sleep(400);
            moveWobbleGoal(WOBBLE_GOAL_UP);

            robot.followTrajectory(traj2);

//going to shooting position (0,-35)

//shooting power shots
            Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(0, -35), Math.toRadians(-180))
                    .build();
            robot.followTrajectory(traj3);
            robot.shootMotorLeft.setPower(0.9);
            robot.autonomousShoot();
            sleep(5000);
            robot.stopAllMotors();
            Trajectory traj4 = robot.trajectoryBuilder(traj3.end())

//parking position
                    .splineTo(new Vector2d(12, -35), Math.toRadians(-180))
                    .build();
            robot.followTrajectory(traj4);
        }

        /* else if (pos == 1) {
            telemetry.addData(">", "Running 1 ring path");
            telemetry.update();

            Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    .splineTo(new Vector2d(-36, -55), Math.toRadians(0))
                    .splineTo(new Vector2d(24, -27), Math.toRadians(45))
                    .build();
            robot.followTrajectory(traj1);

            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            sleep(400);
            robot.openWobbleClaw();
            moveWobbleGoal(WOBBLE_GOAL_UP);

            Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    .splineTo(new Vector2d(-12, -51), Math.toRadians(170))
                    .build();
            robot.shootMotorLeft.setPower(0.8);
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
                    .strafeRight(8)
                    .build();
            robot.followTrajectory(traj4);

            robot.closeWobbleClaw();
            sleep(1000);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(500);
            Trajectory traj5 = robot.trajectoryBuilder(traj4.end())
                    //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                    .splineTo(new Vector2d(24, -27), Math.toRadians(45))
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


        } else {
            telemetry.addData(">", "Running 0 ring path");
            telemetry.update();

            Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
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


        }
*/

    }

}
