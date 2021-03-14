package org.firstinspires.ftc.teamcode.casper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@TeleOp(name = "Casper - Manual Control", group = "Linear Opmode")
public class casperManual extends casperAutonomousBase {
    @Override
    public void runOpMode() {

        robot = new casperMecanumDrive(hardwareMap);
        robot.initVuforia(hardwareMap);

        //This is where we set our motor powers
        double motor_power = 0.3;

        float leftX, leftY, rightZ;

        //robot.setPoseEstimate(PoseStorage.currentPose);
        Pose2d startPose = new Pose2d(-63, -57, Math.toRadians(-0));
        robot.setPoseEstimate(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.update();

            Pose2d myPose = robot.getPoseEstimate();


            // Print your pose to telemetry
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            if (targetVisible) {
                //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                //        translation.get(0) / robot.mmPerInch, translation.get(1) / robot.mmPerInch, translation.get(2) / robot.mmPerInch);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.addData("Pos (in)", "{X, Y, Head} = %.1f, %.1f, %.1f",
                        translation.get(0) / robot.mmPerInch, translation.get(1) / robot.mmPerInch, Math.toRadians(-1*rotation.thirdAngle));
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            }
            telemetry.update();


            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.update();
            //all of the code bellow is setting a power to each button on the gamepad

            if ((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0) || (gamepad1.right_stick_x != 0)) {
                //leftY = gamepad1.left_stick_y;
                //leftX = gamepad1.left_stick_x * -1;
                //rightZ = gamepad1.right_stick_x * -1;
                //robot.moveHolonomic(leftX, leftY, rightZ);
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

            } else if (gamepad1.dpad_down) {
                //forward
                robot.moveHolonomic(0, motor_power * 1, 0);
            } else if (gamepad1.dpad_up) {
                //backwards
                robot.moveHolonomic(0, motor_power * -1, 0);
            } else if (gamepad1.dpad_left) {
                //rotate counter-clockwise
                robot.moveHolonomic(0, 0, motor_power * 1);
            } else if (gamepad1.dpad_right) {
                //rotate clockwise
                robot.moveHolonomic(0, 0, motor_power * -1);
            } else if(gamepad1.right_stick_button) {

            } else if(gamepad1.left_bumper) {
                //st
            } else if (gamepad1.right_bumper) {
                //aligns to position for shooting
                robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
                Trajectory traj2 = robot.trajectoryBuilder(myPose)
                        //.lineToLinearHeading(new Pose2d(-12, -51, Math.toRadians(163)))
                        //.splineTo(new Vector2d(-12, -51), Math.toRadians(163))
                        .splineToLinearHeading(new Pose2d(-12, -51, Math.toRadians(163)), Math.toRadians(0))
                        .build();
                robot.followTrajectory(traj2);
            }
            else if (gamepad1.a) {
                boolean visible = getVuforiaRefPos(1000);
                if (visible)
                {
                    //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    //        translation.get(0) / robot.mmPerInch, translation.get(1) / robot.mmPerInch, translation.get(2) / robot.mmPerInch);
                    //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    //telemetry.addData("Pos (in)", "{X, Y, Head} = %.1f, %.1f, %.1f",
                    //` 11111                                               ````        translation.get(0) / robot.mmPerInch, translation.get(1) / robot.mmPerInch, Math.toRadians(rotation.thirdAngle));
                    Pose2d vPos= new Pose2d(translation.get(0) / robot.mmPerInch, translation.get(1)/ robot.mmPerInch, Math.toRadians(-1*rotation.thirdAngle));
                    //eiihcckgbrrrgfkhhgctunftkrkejnhrudnhftilgicc
                    // robot.setPoseEstimate(vPos);

                }

            }
            else if (gamepad2.dpad_up) {
                robot.collectMotor.setPower(0.9);
            } else if (gamepad2.dpad_down) {
                robot.collectMotor.setPower(-0.9);
            }
            else if (gamepad2.right_bumper){
                robot.collectMotor.setPower(0.8);
                robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
                //robot.grabber.setPosition(0);
            }
            else if (gamepad2.left_bumper){
                robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
            }
            else if (gamepad2.x) {
                robot.closeWobbleClaw();
                //robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
            }
            else if (gamepad2.b) {
                robot.openWobbleClaw();
                //robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
            }
            else if (gamepad2.y){
                robot.wobbleMotor.setPower(0.4);
            }
            else if (gamepad2.a) {
                robot.wobbleMotor.setPower(-0.5);
            } else if (gamepad2.right_stick_button) {
                shooterMotorAuto();
            }else if (gamepad2.dpad_right) {
                robot.closePushRingMore();
                robot.collectMotor.setPower(0.8);
                robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
            } else if (gamepad2.dpad_left) {
                robot.openPushRing();
                robot.collectMotor.setPower(0.8);
                robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
            }
            else {
                robot.stopAllMotors();
            }
            //telemetry.update();
        }
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.update();

    }
}