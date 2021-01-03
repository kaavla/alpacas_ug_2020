package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.TestBot.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "Manav's Odometry OpMode")
public class MyOdometryOpmode extends testBotUtility {
    final double PIVOT_SPEED = 0.4;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    //final double COUNTS_PER_INCH = 307.699557;

    final double COUNTS_PER_ENC_REV  = 8192;
    final double WHEEL_DIAMETER_INCHES = 1.37795;
    final double COUNTS_PER_INCH       = (COUNTS_PER_ENC_REV ) / (WHEEL_DIAMETER_INCHES * 3.1415);

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    double motor_power = 0.3;

    float leftX, leftY, rightZ;

    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHW();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //globalPositionUpdate.reverseRightEncoder();
        //globalPositionUpdate.reverseNormalEncoder();

        //gotoPosition(24*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.3,0,1);
        //sleep(2000);
        //gotoPosition(24*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.3,0,1);
        //sleep(2000);
        //gotoPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.3,0,1);

        while(opModeIsActive()){
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
            } else if (gamepad1.b){
                    robot.shootMotorLeft.setPower(0.6);
                    robot.shootMotorRight.setPower(0.6);
                    robot.collectMotor.setPower(0.8);
            } else {
                robot.stopAllMotors();
            }

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }


    public void gotoPosition(double targetX, double targetY, double power, double finalOrientation, double allowableDistanceError)
    {
        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() && (distance > allowableDistanceError)) {
            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovmentXComponent = calculateX(robotMovementAngle, power);
            double robotMovmentYComponent = calculateY(robotMovementAngle, power);

            double pivotCorrection = finalOrientation - globalPositionUpdate.returnOrientation();

            //Move Robot
            robot.moveHolonomic(robotMovmentYComponent,robotMovmentXComponent,pivotCorrection);

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        }
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
        telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
        telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());
        telemetry.update();


    }


    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

}
