package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.TestPrograms.JARVISAutonomousBase;

import java.util.List;

public class testBotUtility extends LinearOpMode {


    public testBotHW robot = new testBotHW();
    public ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0;
    // public direction;
    public double ref_angle = 0;

    public double distance_traveled = 0;



    @Override
    public void runOpMode() {
        //Empty Function
    }



    public void initHW() {
        RobotLog.ii("CAL", "Enter -  initHW");
        robot.init(hardwareMap);

        telemetry.update();

        RobotLog.ii("CAL", "Exit -  initHW");
    }




}