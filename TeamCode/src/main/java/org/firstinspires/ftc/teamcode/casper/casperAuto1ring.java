package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_UP;

@Autonomous(group = "robot")

public class casperAuto1ring extends casperAutonomousBase {

    public ElapsedTime t1 = new ElapsedTime();
    String timing;


    @Override
    public void runOpMode() throws InterruptedException {
        t1.reset();
        robot = new casperMecanumDrive(hardwareMap);
        //0.9
        timing = String.format("Init Capser Mechanum Drive time = %.03f\n", t1.milliseconds());;
        RobotLog.ii("CASPER", timing);
        t1.reset();

        robot.initVuforia(hardwareMap);
        robot.initTfod(hardwareMap);

        //3.2
        timing = String.format("After initTfod = %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();


//start position
        Pose2d startPose = new Pose2d(-63, -57, Math.toRadians(0));
        robot.setPoseEstimate(startPose);
        //origin at middle of full field(0,0)
        //starting position down red (-60, -48, 90)

        //rings red located at (-24, -36)
        //position for shooting at (-12, -51)
        // position 4 square at (58, -58)
        Trajectory traj0 = robot.trajectoryBuilder(startPose)
                //.splineTo(new Vector2d(-24, -57), Math.toRadians(0))
                .forward(39)
                .build();

        Trajectory traj1 = robot.trajectoryBuilder(traj0.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .splineTo(new Vector2d(24, -35), Math.toRadians(45))
                .build();

        Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                //.lineToLinearHeading(new Pose2d(-12, -51, Math.toRadians(163)))
                //.splineTo(new Vector2d(-12, -51), Math.toRadians(163))
                .splineToLinearHeading(new Pose2d(-12, -51, Math.toRadians(163)), Math.toRadians(0))
                .build();

        Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-27, -24, Math.toRadians(270)), Math.toRadians(0))
                //.splineTo(new Vector2d(-27, -24), Math.toRadians(270))
                .build();

        Trajectory traj4 = robot.trajectoryBuilder(traj3.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .strafeRight(5)
                //.splineTo(new Vector2d(-31, -24), Math.toRadians(270))
                .build();

        Trajectory traj5 = robot.trajectoryBuilder(traj4.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .splineTo(new Vector2d(24, -35), Math.toRadians(45))
                .build();

        Trajectory traj6 = robot.trajectoryBuilder(traj5.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                //.lineToSplineHeading(new Pose2d(12, -40, Math.toRadians(90)))
                .splineTo(new Vector2d(12, -40), Math.toRadians(90))
                .build();
        //0.2
        timing = String.format("After all traj build  = %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

        telemetry.addData(">", "Waiting to start....");
        telemetry.update();
        waitForStart();
        //1.7
        timing = String.format("After waitfor start = %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();
        //going forward to detect rings
        robot.followTrajectory(traj0);

        //2.3
        timing = String.format("After trac0 = %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();
        //int position = 4;
        int pos = getNumRings(1500); //ms
        //telemetry.addData(">", "Num of rings = %d", pos);
        //1.5
        timing = String.format("After getnumrings = %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();
        robot.deinitTfod();


        //telemetry.addData(">", "running 0 ring path");
        //telemetry.update();

//going to drop wobble goal

        robot.followTrajectory(traj1);
        //1.8
        timing = String.format("After tracj1= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

//dropping wobble goal
        robot.shootMotorLeft.setPower(0.8);
        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        sleep(400);
        //1.3
        timing = String.format("After shoot motors= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

//going to shooting position + shooting
        robot.followTrajectory(traj2);
        //1.8
        timing = String.format("After trajj2..move to shoot pos= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

        robot.autonomousShoot();
        sleep(2000);
        //move servo
        //robot.autonomousShoot();
        sleep(1000);
        robot.stopAllMotors();


//replace above with powershot stuff

        //3.0
        timing = String.format("After shooting = %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

//picking up second wobble goal
        robot.followTrajectory(traj3);
        //2.5
        timing = String.format("After traj3..move to pick next goal= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

        robot.followTrajectory(traj4);
        //0.8
        timing = String.format("After traj4..strafe to goal= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

        sleep(100);
        robot.closeWobbleClaw();
        sleep(1000);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        sleep(500);
        //2.7
        timing = String.format("After picj sencond  goal= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

//dropping second wobble goal, going back to same position in traj1
        robot.followTrajectory(traj5);
        //3.2
        timing = String.format("After trcj5 sencond  goal= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();
        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        sleep(200);
        robot.openWobbleClaw();
        sleep(400);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        sleep(300);

        //3.6
        timing = String.format("After drop sencond  goal= %.03f\n", t1.milliseconds());
        RobotLog.ii("CASPER", timing);
        t1.reset();

//park on white line
        //robot.followTrajectory(traj6);
        //1.9
        //timing = String.format("After traj6 = %.03f\n", t1.milliseconds());
        //RobotLog.ii("CASPER", timing);
        //t1.reset();


        telemetry.addData(">", timing);
        telemetry.update();

    }

}