package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.JARVISAutonomousBase;
import org.firstinspires.ftc.teamcode.JARVISHW;

@Autonomous(name="Jarvis Auto Build Site 1 Blue", group="JARVIS")

public class JARVISAutoBLD1Blue extends JARVISAutonomousBase {

    JARVISHW robotJARVIS = new JARVISHW();

    @Override
    public void runOpMode() {
        // Initializes all of the hardware so it can be used in the code.
        robot.init(hardwareMap);
        // Move the foundation attachment up to the start position
        moveFoundationServoUp();

        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();


        // Send telemetry message to signify robot is ready to run;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        // Run the program
        autoBLDFoundation();
    }

    public void autoBLDFoundation()
    {
        RobotLog.ii("CAL", "Enter - JARVISAutoBLD1Blue");

        //initialized the motor encoders
        robot.initMotorEncoders();

        // sure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            //move forward to stop dragging along the wall
            myEncoderDrive(Direction.BACKWARD, 0.1, 2, 5, SensorsToUse.NONE);
            //Strafe left to be in a better position to move the foundation
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 8, 5, SensorsToUse.NONE);
            //move to the foundation
            myEncoderDrive(Direction.BACKWARD, 0.3, 18, 5, SensorsToUse.NONE);
            //go at a slower speed to make sure we hook on properly
            myEncoderDrive(Direction.BACKWARD, 0.07, 7, 5, SensorsToUse.NONE);
            //leave time for the foundation servos to move 
            sleep(500);
            //move the foundation attachment down
            moveFoundationServoDown();
            //leave time for the foundation servos to move
            sleep(500);


            //move backwards with the foundation and bring it close to the wall
            myEncoderDrive(Direction.FORWARD, 0.2, 31, 5, SensorsToUse.NONE);
            //leave time for the robot to finish turning
            sleep(500);
            //move the foundation attachment up to release the foundation
            moveFoundationServoUp();
            //stop for 10 seconds so the robot is out of the way of the other robots while the
            //autonomous mode is still going on
            sleep(8000);
            //move left to be right next to the foundation
            myEncoderDrive(Direction.STRAFE_LEFT, 0.3, 50, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD1Blue");
    }

}