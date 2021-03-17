package org.firstinspires.ftc.teamcode.smartW;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "Smart WheelChair")
//@Disabled
public class smartWOpmode extends smartWUtility {

    @Override
    public void runOpMode() {
        initHW();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive() && !isStopRequested()){
            xPos    = globalPositionUpdate.returnXCoordinate() ;
            yPos    = globalPositionUpdate.returnYCoordinate();
            heading = globalPositionUpdate.returnOrientation();

            //Display Global (x, y, theta) coordinates
            telemetry.addData("{X, Y, Thetha} : ", "%2.2f, %2.2f, %2.2f", xPos, yPos, heading);
            if (debug) {
                telemetry.addData("VL Pos : %d", robot.verticalLeft.getCurrentPosition());
                telemetry.addData("VR Pos : %d", robot.verticalRight.getCurrentPosition());
                telemetry.addData("H Pos  : %d", robot.horizontal.getCurrentPosition());
                telemetry.addData("Thread Active", positionThread.isAlive());
            }
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

}
