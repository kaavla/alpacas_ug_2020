package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_UP;


    @Autonomous(group = "robot")

    public class Autonomous4powershot extends casperAutonomousBase {

        public ElapsedTime t1 = new ElapsedTime();
        String timing;


        @Override
        public void runOpMode() throws InterruptedException {
            robot = new casperMecanumDrive(hardwareMap);
            robot.initVuforia(hardwareMap);
            robot.initTfod(hardwareMap);

            CLOSE_RING();
            Pose2d startPose = new Pose2d(-63, -57, Math.toRadians(0));
            robot.setPoseEstimate(startPose);

            Trajectory traj0 = robot.trajectoryBuilder(startPose)
                    .forward(39)
                    .build();

            telemetry.addData(">", "Waiting to start....");
            telemetry.update();

            waitForStart();
            //going forward to detect rings
            robot.followTrajectory(traj0);

            //2.3
            int pos = getNumRings(1500); //ms
            telemetry.addData(">", "Num of rings = %d", pos);
            //1.5


//going to drop wobble goal
/*
            Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
                    //.splineToLinearHeading(new Pose2d(50, -50, Math.toRadians(0)), Math.toRadians(0))
                    .splineTo(new Vector2d(50, -50), Math.toRadians(0))
                    .build();
            robot.followTrajectory(traj1);

//dropping wobble goal
            robot.shootMotorLeft.setPower(0.8);
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            sleep(400);


            //1.3
        /*
        Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
                //.splineToLinearHeading(new Pose2d(50, -50, Math.toRadians(0)), Math.toRadians(0))
                //.back(65)
                .line
                .build();
        robot.followTrajectory(traj11);


//going to shooting position + shooting
            Trajectory traj2 = robot.trajectoryBuilder(traj1.end(), true)
                    .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(163)), Math.toRadians(0))
                    .build();

            //1.8

            robot.autonomousShoot();
            sleep(2000);
            //move servo
            //robot.autonomousShoot();
            sleep(1000);
            robot.stopAllMotors();

            Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                    .strafeLeft(14)
                    .build();

            robot.followTrajectory(traj3);
            Trajectory traj31 = robot.trajectoryBuilder(traj3.end())
                    .forward(18)
                    .build();
            OPEN_RING();
            robot.collectMotor.setPower(0.7);
            sleep(500);

            robot.followTrajectory(traj31);


/*
            //3.0
//picking up second wobble goal
            Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                    .splineToLinearHeading(new Pose2d(-25, -25, Math.toRadians(265)), Math.toRadians(0))
                    .build();
            robot.followTrajectory(traj3);
            //2.5

            Trajectory traj4 = robot.trajectoryBuilder(traj3.end())
                    .strafeRight(5)
                    .build();
            robot.followTrajectory(traj4);
            //0.8

            sleep(100);
            robot.closeWobbleClaw();
            sleep(1000);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(500);
            //2.7

//dropping second wobble goal, going back to same position in traj1
            Trajectory traj5 = robot.trajectoryBuilder(traj4.end())
                    //.splineToLinearHeading(new Pose2d(50, -50, Math.toRadians(0)), Math.toRadians(0))
                    .splineTo(new Vector2d(50, -50), Math.toRadians(0))
                    .build();
            robot.followTrajectory(traj5);
            //3.2
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            sleep(200);
            robot.openWobbleClaw();
            sleep(400);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(300);
            OPEN_RING();

            //3.6

//park on white line

            Trajectory traj4 = robot.trajectoryBuilder(traj31.end())
                    .splineTo(new Vector2d(12, -40), Math.toRadians(90))
                    .build();
            robot.followTrajectory(traj4);
            //1.9
            //timing = String.format("After traj6 = %.03f\n", t1.milliseconds());
            //RobotLog.ii("CASPER", timing);
            //t1.reset();


            telemetry.addData(">", timing);
            telemetry.update();
            robot.deinitTfod();

*/
        }

    }

