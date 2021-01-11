package org.firstinspires.ftc.teamcode.TestBot;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

    @Autonomous(group = "drive")
    public class AutonomousRemote extends testBotAutonomousBase {
        @Override

        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(-60, -24, Math.toRadians(90));
            drive.setPoseEstimate(startPose);


            waitForStart();
            int position = 0;

            if (isStopRequested()) return;


            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-24, -36), Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj1);


            if (position == 0) {
                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .splineTo(new Vector2d(-6, -36), Math.toRadians(90))
                        .build();
                drive.followTrajectory(traj2);
            } else if (position == 1) {
                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .splineTo(new Vector2d(24, -12), Math.toRadians(90))
                        .build();
                drive.followTrajectory(traj2);
            } else if (position == 4) {
                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                       .splineTo(new Vector2d(48, -42), Math.toRadians(45))
                     .build();
            }
            //traj3 in position to shoot rings into the upper target, and then return to get wobble
            Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                    //exact position to shoot rings into the upper goal
                    .splineTo(new Vector2d(0, -22), Math.toRadians(202))
                    .build();
            drive.followTrajectory(traj3);

            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .splineTo(new Vector2d(-30, -4), Math.toRadians(-90))
                    .build();
            drive.followTrajectory(traj4);
            sleep(1000);
            myWobbleGoal(Direction.SLIDE_DOWN,0.5,1,4,SensorsToUse.NONE);

        }

    }

