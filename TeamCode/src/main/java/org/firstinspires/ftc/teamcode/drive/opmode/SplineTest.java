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
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-60, -24, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
                .build();
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
        drive.followTrajectory(traj1);
        sleep(300);
    }

}
