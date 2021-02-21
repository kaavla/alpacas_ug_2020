package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.casper.casperAutonomousBase.wobbleGoalMode.WOBBLE_GOAL_UP;

//origin at0 middle of full field(0,0)
//starting position down red (-60, -48, 90)
//rings red located at (-24, -36)
//position for shooting at (-12, -51)
// position 4 square at (58, -58)

@Autonomous(group = "robot")
public class casperAutoV2 extends casperAutonomousBase {

    public Trajectory traj0, traj1, traj2, traj3, traj4,traj5, traj6;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new casperMecanumDrive(hardwareMap);

        robot.initVuforia(hardwareMap);
        robot.initTfod(hardwareMap);

        //Set start position
        Pose2d startPose = new Pose2d(-56, -48, Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        traj0 = robot.trajectoryBuilder(startPose)
                .strafeRight(9)
                .build();

        traj1 = robot.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(-36, -55), Math.toRadians(0))
                .splineTo(new Vector2d(50, -50), Math.toRadians(45))
                .build();

        traj2 = robot.trajectoryBuilder(traj1.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .splineTo(new Vector2d(-12, -51), Math.toRadians(170))
                .build();

        traj3 = robot.trajectoryBuilder(traj2.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                .lineToSplineHeading(new Pose2d(-25, -24, Math.toRadians(270)))
                .build();

        traj4 = robot.trajectoryBuilder(traj3.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .strafeRight(8)
                .build();

        traj5 = robot.trajectoryBuilder(traj4.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                .splineTo(new Vector2d(50, -50), Math.toRadians(45))
                .build();

        traj6 = robot.trajectoryBuilder(traj5.end())
                //.lineToLinearHeading(new Pose2d(-36, -55, Math.toRadians(0)))
                //.splineTo(new Vector2d(-28, -24), Math.toRadians(270))
                .lineToSplineHeading(new Pose2d(12, -40, Math.toRadians(90)))
                .build();


        waitForStart();
        //if (isStopRequested()) return;

        robot.followTrajectory(traj1);

        //detecting rings
        int pos = getNumRings(1000); //ms
        telemetry.addData(">", "Num of rings = %d", pos);
        telemetry.update();

        //moving to drop the wobble goal?
   //     if (pos == 4){
            robot.followTrajectory(traj1);

            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            moveWobbleGoal(WOBBLE_GOAL_UP);

            //moving behind the start (white line) to shoot the rings?
            robot.shootMotorLeft.setPower(0.7);

            robot.followTrajectory(traj2);
            robot.autonomousShoot();
            sleep(5000);
            robot.stopAllMotors();

            //picking up the 2nd wobble goal?
            robot.followTrajectory(traj3);
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            robot.followTrajectory(traj4);
            robot.closeWobbleClaw();
            sleep(1000);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            sleep(500);

            robot.followTrajectory(traj5);
            moveWobbleGoal(WOBBLE_GOAL_DOWN);
            moveWobbleGoal(WOBBLE_GOAL_UP);
            robot.followTrajectory(traj6);
 //       }
        robot.deinitTfod();
    }

    public void doFourRingPath(Pose2d startPose){
        telemetry.addData(">", "Num of rings = 4");
        telemetry.update();


        robot.followTrajectory(traj1);

        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        moveWobbleGoal(WOBBLE_GOAL_UP);

        //moving behind the start (white line) to shoot the rings?
        robot.shootMotorLeft.setPower(0.7);

        robot.followTrajectory(traj2);
        robot.autonomousShoot();
        sleep(5000);
        robot.stopAllMotors();

        //picking up the 2nd wobble goal?
        robot.followTrajectory(traj3);
        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        robot.followTrajectory(traj4);
        robot.closeWobbleClaw();
        sleep(1000);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        sleep(500);

        robot.followTrajectory(traj5);
        moveWobbleGoal(WOBBLE_GOAL_DOWN);
        moveWobbleGoal(WOBBLE_GOAL_UP);
        robot.followTrajectory(traj6);
    }

}



