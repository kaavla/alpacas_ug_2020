package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "Servo Encoder")
@Disabled
public class ServoEncoder extends testBotUtility {


    final double COUNTS_PER_ENC_REV  = 8192;

    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHW();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            double tickPerDegree = 8192 / 360;
            double currentservoDegree = robot.servoEncoder.getCurrentPosition()/tickPerDegree;
            double max = 8192/4;
            double min = -8192/4;

            double targetServoDegree = 60;

            if(currentservoDegree >= max || currentservoDegree <= min) {
                robot.servoTurret.setPower(0);
            }
            else {
                if(currentservoDegree==targetServoDegree){
                    robot.servoTurret.setPower(0);
                }
                else{
                    robot.servoTurret.setPower(1);
                }
            }
            telemetry.addData("Turret servo encoder position", robot.servoEncoder.getCurrentPosition());
            telemetry.update();
        }

        //Stop the thread
    }

}
