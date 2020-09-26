package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.JARVISAutonomousBase;
import org.firstinspires.ftc.teamcode.JARVISHW;

@Autonomous(name="ZZZZZ2", group="JARVIS")
@Disabled
public class JARVISAutoBLDBlueParkMiddle extends JARVISAutonomousBase {

    JARVISHW robotJARVIS = new JARVISHW();

    @Override
    public void runOpMode() {
        // Initializes the motors so they are ready for use.
        robot.init(hardwareMap);
        // move the foundation attachment up to the start position
        moveFoundationServoUp();

        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();


        // Send telemetry message to signify robot waiting;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        // Runs the program
        autoBLDFoundation();
    }

    public void autoBLDFoundation()
    {
        RobotLog.ii("CAL", "Enter - JARVISAutoBLD2Blue");

        //initialized the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            //move Sideways into the middle
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.5, 15, 5, SensorsToUse.NONE);
            //move forward to the line
            myEncoderDrive(Direction.BACKWARD, 0.5, 25, 5, SensorsToUse.NONE);
            //strafe further out of the way
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.5, 4, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD2Blue");
    }

}