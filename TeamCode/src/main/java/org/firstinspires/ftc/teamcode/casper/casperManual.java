package org.firstinspires.ftc.teamcode.casper;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Casper - Manual Control", group = "Linear Opmode")
public class casperManual extends casperAutonomousBase {
    @Override
    public void runOpMode() {
        robot = new casperMecanumDrive(hardwareMap);
        //This is where we set our motor powers
        double motor_power = 0.3;

        float leftX, leftY, rightZ;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //all of the code bellow is setting a power to each button on the gamepad

            if ((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0) || (gamepad1.right_stick_x != 0)) {
                leftY = gamepad1.left_stick_y;
                leftX = gamepad1.left_stick_x * -1;
                rightZ = gamepad1.right_stick_x * -1;
                robot.moveHolonomic(leftX, leftY, rightZ);
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
                //insert code to make the robot align to shooting position
            } else if(gamepad1.left_bumper) {
                //strafe left
            } else if (gamepad1.right_bumper) {
                //strafe right
            }

            else if (gamepad2.dpad_up) {
                robot.collectMotor.setPower(0.7);
            } else if (gamepad2.dpad_down) {
                robot.collectMotor.setPower(-0.7);
            }
            else if (gamepad2.right_bumper){
                robot.collectMotor.setPower(0.8);
                robot.shootMotorLeft.setPower(0.7);
                //robot.grabber.setPosition(0);
            }
            else if (gamepad2.left_bumper){
                robot.shootMotorLeft.setPower(0.7);
            }
            else if (gamepad2.x) {
                robot.closeWobbleClaw();
            }
            else if (gamepad2.b) {
                robot.openWobbleClaw();
            }
            else if (gamepad2.y){
                //robot.collectMotor.setPower(0.8);
                robot.wobbleMotor.setPower(0.4);
            }
            else if (gamepad2.a) {
                //robot.collectMotor.setPower(-0.5);
                robot.wobbleMotor.setPower(-0.4);
            }
            else {
                robot.stopAllMotors();
            }
            telemetry.update();
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}