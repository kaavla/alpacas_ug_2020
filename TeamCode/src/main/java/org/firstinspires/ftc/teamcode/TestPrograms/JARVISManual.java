package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "JARVIS Manual", group = "Linear Opmode")
@Disabled
public class JARVISManual extends JARVISAutonomousBase {
    @Override
    public void runOpMode() {
        //This is where we set our motor powers
        double motor_power = 0.3;

        float leftX, leftY, rightZ;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initHW();
        robot.openCapStoneClaw();

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
            } else if (gamepad1.x) {
                robot.openGrabberClaw(0);
                robot.openGrabberClaw(1);
            } else if (gamepad1.b) {
                robot.closeGrabberClaw(0);
                robot.closeGrabberClaw(1);
            } else if (gamepad1.y) {
                robot.setGrabberDown(0);
                robot.setGrabberDown(1);
            } else if (gamepad1.a) {
                robot.setGrabberUp(0);
                robot.setGrabberUp(1);
           } else if (gamepad1.right_trigger > 0.5) {
                //StrafeRight slow
                robot.moveHolonomic(-0.4, 0, 0);
            } else if (gamepad1.left_trigger > 0.5) {
                //StrafeLeft slow
                robot.moveHolonomic(0.4, 0, 0);
            } else if (gamepad1.left_bumper){
                robot.moveFoundationServoUp();
            } else if (gamepad1.right_bumper) {
                robot.moveFoundationServoDown();
            } else if (gamepad2.dpad_up) {
                robot.slidesUp(0.8);
                //myEncoderSlide1(Direction.SLIDE_UP, 0.9, 6, 4, SensorsToUse.NONE); //moves the slide up 6 inches every time
            } else if (gamepad2.dpad_down) {
                //robot.closeClaw(); //closes the claw
                robot.slidesDown(0.2);
                //myEncoderSlide1(Direction.SLIDE_DOWN, 0.9, 6, 4, SensorsToUse.NONE); //moves the slide down 6 inches every time
           } else if (gamepad2.dpad_left) {
                robot.slideIn(0.5);
            } else if (gamepad2.dpad_right) {
                robot.slideOut(0.5);
            } else if (gamepad2.x) {
                robot.openClaw();  //opens the claw
            } else if (gamepad2.b) {
                robot.closeClaw(); //closes the claw
            } else if (gamepad2.y) {
                robot.rotateClawPerpendicular();  //claw perpendicular
            } else if (gamepad2.a) {
                robot.rotateClawInline(); //claw inline
            } else if (gamepad2.left_trigger > 0.5){
                robot.resetCollectionServo();
            } else if (gamepad2.right_trigger > 0.5) {
                robot.openClaw();  //Automatically opens the claw
                robot.setCollectionServo();
            } else if (gamepad2.left_bumper){
                robot.closeCapStoneClaw();
            } else if (gamepad2.right_bumper) {
                robot.openCapStoneClaw();
            } else if (gamepad2.left_stick_y > 0) {
                robot.measureTapeOut();
            } else if (gamepad2.left_stick_y < 0) {
                robot.measureTapeIn();
            }  else{
                robot.stopAllMotors();
            }
            if (robot.touch_sensor.isPressed() ) {
                telemetry.addData("TOUCH SENSOR IS PRESSED", "THE BLOCK IS IN THE CLAW - YOU CAN NOW CLOSE THE CLAW");
                //robot.closeClawSensor(10); //closes the claw
            } else {
                telemetry.addData("not", "pressed");
            }

            telemetry.update();
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}







