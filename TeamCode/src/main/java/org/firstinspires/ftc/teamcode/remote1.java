package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

    @Autonomous(group = "drive")
    public class remote1 extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(-60, -24, Math.toRadians(90));
            drive.setPoseEstimate(startPose);

            waitForStart();

            if (isStopRequested()) return;


            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-24, -36), Math.toRadians(90))
                    .build();
            //one ring
           // Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
             //       .splineTo(new Vector2d(24, -12), Math.toRadians(90))
               //     .build();
            drive.followTrajectory(traj1);
            //four rings
           // Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
             //       .splineTo(new Vector2d(48, -42), Math.toRadians(45))
               //     .build();
            //zero rings, set first traj Math.toRadians(90) for this, other two paths set at 0
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .strafeRight(24)
                   .build();
            drive.followTrajectory(traj2);


        }

    }

