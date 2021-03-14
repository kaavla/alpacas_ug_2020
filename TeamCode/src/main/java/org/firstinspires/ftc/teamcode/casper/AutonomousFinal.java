package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_UP;

@Autonomous(group = "robot")

public class AutonomousFinal extends casperAutonomousBase {

    public ElapsedTime t1 = new ElapsedTime();
    String timing;
    Trajectory traj0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new casperMecanumDrive(hardwareMap);
        robot.initVuforia(hardwareMap);
        robot.initTfod(hardwareMap);

        //start position
        closePushRing();
        Pose2d startPose = new Pose2d(-63, -57, Math.toRadians(0));
        robot.setPoseEstimate(startPose);

        traj0 = robot.trajectoryBuilder(startPose)
                .forward(39)
                .build();

        telemetry.addData(">", "Waiting to start....");
        telemetry.update();
        waitForStart();

        //going forward to detect rings
        robot.followTrajectory(traj0);
        int rings = getNumRings(1500); //ms
        telemetry.addData(">", "Rings Detected = %d", rings);
        telemetry.update();

        robot.deinitTfod();

        if(rings==0) {
            runZeroRings();
        } else if (rings==1) {
            runOneRings();
        } else if (rings==4) {
            runFourRings();
        }
        PoseStorage.currentPose = robot.getPoseEstimate();
    }

    public void runZeroRings(){
        Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(0, -50), Math.toRadians(45))
                .build();

        Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(-12, -51, Math.toRadians(164)), Math.toRadians(0))
                .build();

        Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-25, -25, Math.toRadians(265)), Math.toRadians(0))
                .build();

        Trajectory traj4 = robot.trajectoryBuilder(traj3.end())
                .strafeRight(4.8)
                .build();

        Trajectory traj5 = robot.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(0, -50), Math.toRadians(45))
                .build();

        Trajectory traj6 = robot.trajectoryBuilder(traj5.end())
                .splineTo(new Vector2d(12, -40), Math.toRadians(90))
                .build();

        //Move to point to drop wobble goal
        robot.followTrajectory(traj1);

        //Start Motore to get a lead time to reach given speed
        startShootMotor();

        //Drop wobble goal
        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        operateWobbleClaw(WOBBLE_GOAL_CLAW_OPEN);

        //move to shooting position + shooting
        robot.followTrajectory(traj2);

        //Start Motore to get a lead time to reach given speed
        startShootMotor();

        sleep(1000);
        robot.autonomousShoot();
        sleep(2000);
        robot.closePushRingMore();
        sleep(1000);
        robot.stopAllMotors();

        //replace above with powershot stuff in the future sometime

        //Move to picking up second wobble goal
        robot.followTrajectory(traj3);

        //Strafe a bit forward to catch the wobble goal
        robot.followTrajectory(traj4);

        sleep(100);
        operateWobbleClaw(WOBBLE_GOAL_CLAW_CLOSE);
        moveWobbleGoal(WOBBLE_GOAL_UP);

        //Mopve to drop 2nd wobble goal
        robot.followTrajectory(traj5);

        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        operateWobbleClaw(WOBBLE_GOAL_CLAW_OPEN);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        openPushRing();

        PoseStorage.currentPose = robot.getPoseEstimate();
    }

    public void runOneRings(){
        //going to drop wobble goal
        //TRAJECTORIES 7-12 ARE FOR 1 RING
        Trajectory traj7 = robot.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(24, -35), Math.toRadians(45))
                .build();

        Trajectory traj8 = robot.trajectoryBuilder(traj7.end())
                .splineToLinearHeading(new Pose2d(-12, -51, Math.toRadians(165)), Math.toRadians(0))
                .build();

        Trajectory traj9 = robot.trajectoryBuilder(traj8.end())
                .splineToLinearHeading(new Pose2d(-25, -25, Math.toRadians(265)), Math.toRadians(0))
                .build();

        Trajectory traj10 = robot.trajectoryBuilder(traj9.end())
                .strafeRight(4.8)
                .build();

        Trajectory traj11 = robot.trajectoryBuilder(traj10.end())
                .splineTo(new Vector2d(24, -35), Math.toRadians(45))
                .build();

        Trajectory traj12 = robot.trajectoryBuilder(traj11.end())
                .splineTo(new Vector2d(12, -40), Math.toRadians(90))
                .build();

        //Move to point to drop wobble goal
        robot.followTrajectory(traj7);

        //dropping wobble goal
        startShootMotor();
        moveWobbleGoal(WOBBLE_GOAL_DOWN);

        //going to shooting position + shooting
        robot.followTrajectory(traj8);

        startShootMotor();
        sleep(1000);
        robot.autonomousShoot();
        sleep(2000);
        robot.closePushRingMore();
        sleep(1000);
        robot.stopAllMotors();


        //replace above with powershot stuff

        //picking up second wobble goal
        robot.followTrajectory(traj9);

        robot.followTrajectory(traj10);

        sleep(100);
        operateWobbleClaw(WOBBLE_GOAL_CLAW_CLOSE);
        moveWobbleGoal(WOBBLE_GOAL_UP);

        //dropping second wobble goal, going back to same position in traj1
        robot.followTrajectory(traj11);

        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        operateWobbleClaw(WOBBLE_GOAL_CLAW_OPEN);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        openPushRing();

        telemetry.addData(">", timing);
        telemetry.update();

        PoseStorage.currentPose = robot.getPoseEstimate();
    }

    public void runFourRings(){
        Trajectory traj14 = robot.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(50, -50), Math.toRadians(0))
                .build();
        Trajectory traj15 = robot.trajectoryBuilder(traj14.end(), true)
                .splineToLinearHeading(new Pose2d(-12, -51, Math.toRadians(164)), Math.toRadians(0))
                .build();
            /*
            Trajectory traj16 = robot.trajectoryBuilder(traj15.end())
                    .splineToLinearHeading(new Pose2d(-25, -25, Math.toRadians(265)), Math.toRadians(0))
                    .build();
            Trajectory traj17 = robot.trajectoryBuilder(traj16.end())
                    .strafeRight(5)
                    .build();
            Trajectory traj18 = robot.trajectoryBuilder(traj17.end())
                    .splineTo(new Vector2d(50, -50), Math.toRadians(0))
                    .build();

             */
        Trajectory traj19 = robot.trajectoryBuilder(traj15.end())
                .splineTo(new Vector2d(12, -40), Math.toRadians(90))
                .build();

        robot.followTrajectory(traj14);

        //dropping wobble goal
        startShootMotor();
        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        //1.3
        /*
        Trajectory traj11 = robot.trajectoryBuilder(traj0.end())
                //.splineToLinearHeading(new Pose2d(50, -50, Math.toRadians(0)), Math.toRadians(0))
                //.back(65)
                .line
                .build();
        robot.followTrajectory(traj11);
        */

    //going to shooting position + shooting

        robot.followTrajectory(traj15);
        //1.8

        startShootMotor();
        sleep(1000);
        robot.autonomousShoot();
        sleep(2000);
        robot.closePushRingMore();
        sleep(1000);
        robot.stopAllMotors();

        robot.followTrajectory(traj19);


        //replace above with powershot stuff

        //3.0
        //picking up second wobble goal
        //robot.followTrajectory(traj16);
        //2.5

        //robot.followTrajectory(traj17);
        //0.8
            /*
            sleep(100);
            robot.closeWobbleClaw();
            sleep(1000);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(500);
            //2.7


            //dropping second wobble goal, going back to same position in traj1
            robot.followTrajectory(traj18);

            //3.2
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            sleep(200);
            robot.openWobbleClaw();
            sleep(400);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(300);

             */
        openPushRing();

        //3.6

        //park on white line


        //robot.followTrajectory(traj6);
        //1.9
        //timing = String.format("After traj6 = %.03f\n", t1.milliseconds());
        //RobotLog.ii("CASPER", timing);
        //t1.reset();


        telemetry.addData(">", timing);
        telemetry.update();

        PoseStorage.currentPose = robot.getPoseEstimate();

    }

}