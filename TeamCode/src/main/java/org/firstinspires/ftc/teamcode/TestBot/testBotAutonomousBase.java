package org.firstinspires.ftc.teamcode.TestBot;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.TestBot.testBotHW;

import java.util.List;
import java.util.Locale;

//@Disabled
public class testBotAutonomousBase extends LinearOpMode {

    public enum Direction {
        FORWARD, BACKWARD, STRAFE_RIGHT, STRAFE_LEFT, SLIDE_UP, SLIDE_DOWN, SLIDE_IN, SLIDE_OUT, DIAGONAL_LEFT, DIAGONAL_RIGHT;
    }

    public enum SensorsToUse {
        NONE, USE_COLOR_LEFT, USE_COLOR_RIGHT, USE_DISTANCE_LEFT, USE_DISTANCE_RIGHT, USE_TOUCH,
        USE_DISTANCE_RIGHT_BLD, USE_DISTANCE_LEFT_BLD, USE_DISTANCE_LEFT_FDT, USE_DISTANCE_RIGHT_FDT,
        USE_DISTANCE_FRONT;
    }

    public enum SideToUse {
        USE_LEFT, USE_RIGHT;
    }

    public testBotHW robot = new testBotHW();
    public ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0;
    // public direction;
    public double ref_angle = 0;
    public double ref_angle_1 = 0;

    public Direction direction;

    public double distance_traveled = 0;

    public static final double TICKS_PER_REV = 383.6;
    public static final double MAX_RPM = 435;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 1.9685*2;     // For figuring circumference
    static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double PULLEY_COUNTS_PER_INCH = (19.2 * 28) / (1 * 3.1415); //gobilda 5202 312 rpm motors
    //static final double INOUT_COUNTS_PER_INCH = (19.2 * 28) / (2 * 3.1415); //gobilda 5202 117 rpm motors
    public static double WHEEL_RADIUS = 1.9685; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    //public static double TRACK_WIDTH = 17.66; // in
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.7;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    final String VUFORIA_KEY = "ATVrdOT/////AAABmegFa9L6UUB2ljwRjEStPmU7NS6gi/+GLAe6uAv7o+cB7+pj9EORNLk32cxovTaRj+rUeNw75EMjs5jM0K2OlNn8iO861FyZ5bqnHeBQRr/tR4NIZkQq4ak2zpPLQyyGFzhEkHjnhenYh0dyvxluXF79u8VwJ+g77slCyrCjvgMp6VfEAPLpVJmjzq4hRJMtjYpoRp/agnYFU8HVnmQeGRbjKi1PHLbhP98IkGMowt6Hlobdd2l0vt7msVhwNombHz0XcwJEjwnRKoOkeg7s+kIWvd5paYiO/bnClo9DahFboEFWw1/9wutXgI6/7AGcvwZzkk1HwRh3qZRAWNUSq1hrcjdq9f2QXAYyiqd3wLpT";


    public TFObjectDetector tfod = null;
    public VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Empty Function
    }


    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            40.0, 40.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    private void initVuforia() {
        RobotLog.ii("CAL", "Enter -  initVuforia");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        RobotLog.ii("CAL", "Exit -  initVuforia");
        telemetry.addData("Path1", "Init Vuforia Done");
        telemetry.update();
    }

    private void initTfod() {
        RobotLog.ii("CAL", "Enter -  initTfod");
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        RobotLog.ii("CAL", "Exit -  initTfod");
        telemetry.addData("Path1", "initTfod Done");
        telemetry.update();
    }

    public void initHW() {
        RobotLog.ii("CAL", "Enter -  initHW");
        robot.init(hardwareMap);
        telemetry.addData("Path1", "Init HW Done");
        telemetry.update();

        RobotLog.ii("CAL", "Exit -  initHW");
    }

    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotLog.ii("CAL", "resetAngle - lastAngles = %2.2f", lastAngles.firstAngle);
        globalAngle = 0;
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double getAbsoluteAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


        }
    
