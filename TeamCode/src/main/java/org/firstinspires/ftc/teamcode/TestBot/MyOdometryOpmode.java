package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.TestBot.OdometryGlobalCoordinatePosition;

@TeleOp(name = "Manav's Odometry OpMode")
//@Disabled
public class MyOdometryOpmode extends testBotUtility {

    //number of counts the encoder reads per revolution
    final double COUNTS_PER_ENC_REV  = 8192;
    //diameter of wheel conected to encoder
    final double WHEEL_DIAMETER_INCHES = 1.37795;
    //counts per inch is calculated by taking your counts given by the encoder and dividing it by the circumference of the wheel
    final double COUNTS_PER_INCH       = (COUNTS_PER_ENC_REV ) / (WHEEL_DIAMETER_INCHES * 3.1415);

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHW();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

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
