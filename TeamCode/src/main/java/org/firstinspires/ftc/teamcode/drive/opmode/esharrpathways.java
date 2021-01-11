package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.eiihcckgbrrrllbrrllkcjjnnlerlfvtunggvvfcnhci
 *
 */
@Autonomous(group = "drive")
public class esharrpathways extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-60, -24, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        /*Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(20, 0, Math.toRadians(90)), Math.toRadians(0))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(0, -20, Math.toRadians(0)), Math.toRadians(90))
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2); */

        // goes to (20,0) then (0,-20) medium speed

        //Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                //.lineToSplineHeading(new Pose2d(20, 0, Math.toRadians(90)))
                //.build();

       Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(20, 0), Math.toRadians(0))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(0, -20), Math.toRadians(0))
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
//faster, less accurate, could use in simple autonomous with wobble goal
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(20, 0), 0)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(0, -20), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        /*Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(10)
                .build();
        drive.followTrajectory(traj5);*/
        //turns too much, slower


        /*Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(0, -10), Math.toRadians(90)), Math.toRadians(0))
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2); */
        /*
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(neweiihcckgbrrrilbbievnvjevkgvgvejvkclfhbruufkb
                 Pose2d(-35, -5, Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(0, -26), 0)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-20, -12), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj1);
        sleep(300);
        drive.followTrajectory(traj2);
        sleep(300);
        drive.followTrajectory(traj3);
        sleep(300);
        drive.followTrajectory(traj4);
        sleep(300);
        // sleep(5000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj4.end(), false)
                        .splineTo(new Vector2d(0, -12), Math.toRadians(0))
                        .build()
        );
*/
        //drive.followTrajectory(traj1);
        //sleep(300);
    }

}
