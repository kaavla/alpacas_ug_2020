package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "M2", rbName = "M4", lfName = "M1", lbName = "M3";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    final double PIVOT_SPEED = 0.5;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 307.699557;
    //final double COUNTS_PER_MOTOR_REV  = 360;
    //final double WHEEL_DIAMETER_INCHES = 1.49606;
    //final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV ) / (WHEEL_DIAMETER_INCHES * 3.1415);

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        gotoPosition(36*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5,0,1);

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void moveHolonomic(double x, double y , double z)
    {
        double max_power = 1;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x - z, min_power, max_power);
        double fr_power = Range.clip(y - x + z, min_power, max_power);
        double br_power = Range.clip(y + x + z, min_power, max_power);
        double bl_power = Range.clip(y - x - z, min_power, max_power);

        // Sets the power of the motors to the power defined above
        left_front.setPower(fl_power);
        right_front.setPower(fr_power);
        left_back.setPower(bl_power);
        right_back.setPower(br_power);


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
            moveHolonomic(robotMovmentYComponent,robotMovmentXComponent,pivotCorrection);

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        }
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Hardware Map Init Complete");
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
