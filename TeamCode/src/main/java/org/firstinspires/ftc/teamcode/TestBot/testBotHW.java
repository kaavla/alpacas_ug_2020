package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class testBotHW {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor backrightMotor = null;
    public DcMotor backleftMotor = null;
    public DcMotor collectMotor = null;
    public DcMotor shootMotorRight = null;
    public DcMotor shootMotorLeft = null;
    public Servo grabber = null;
    public BNO055IMU imu = null;

    DcMotorEx verticalLeft, verticalRight, horizontal;

    Orientation lastAngles = new Orientation();  //?
    double globalAngle, power = .40, correction;  //?

    //sets the power used in each of the actions

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        RobotLog.ii("CAL", "Enter - init");


        leftMotor = ahwMap.get(DcMotor.class, "M1");
        rightMotor = ahwMap.get(DcMotor.class, "M2");
        backleftMotor = ahwMap.get(DcMotor.class, "M3");
        backrightMotor = ahwMap.get(DcMotor.class, "M4");
        collectMotor = ahwMap.get(DcMotor.class, "M5");
        shootMotorRight = ahwMap.get(DcMotor.class, "M6");
        shootMotorLeft = ahwMap.get(DcMotor.class, "M7");
        grabber = ahwMap.get(Servo.class, "grabber");
        RobotLog.ii("CAL", "Enter - DC Motor Initialized");

        verticalLeft = ahwMap.get(DcMotorEx.class, "M1");
        verticalRight = ahwMap.get(DcMotorEx.class, "M2");
        horizontal = ahwMap.get(DcMotorEx.class, "M3");
        RobotLog.ii("CAL", "Enter - Encoder  Initialized");
        //verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //eiihcckgbrrrbreucrhbfdiigeenglcfehcefhcjfegn
        // horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = ahwMap.get(BNO055IMU.class, "imu 1");

        //initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        RobotLog.ii("CAL", "Enter - IMU  Initialized");

        //Invert direction for left motors
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RobotLog.ii("CAL", "Enter -Directions reversed");

        // Set all motors to zero power
        stopAllMotors();
        RobotLog.ii("CAL", "Stop all motors");

        initMotorNoEncoders();
        RobotLog.ii("CAL", "No encoders");

        initOdodmetryEncoders();
        RobotLog.ii("CAL", "init  encoders");

        RobotLog.ii("CAL", "Exit - init");

    }

    //resets the power to zero before starting the action
    public void stopAllMotors() {
        RobotLog.ii("CAL", "Stopping All motors");
        leftMotor.setPower(0);
        rightMotor.setPower(0);
      backleftMotor.setPower(0); collectMotor.setPower(0);
        backrightMotor.setPower(0);
        shootMotorLeft.setPower(0);
        shootMotorRight.setPower(0);
    }

    public void initMotorNoEncoders() {
        RobotLog.ii("CAL", "Enter -  initMotorNoEncoders");

        // Sets the mode of the motors to run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collectMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set zero power behavior to braking
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        RobotLog.ii("CAL", "Exit -  initMotorNoEncoders");
    }

    public void initOdodmetryEncoders() {
        RobotLog.ii("CAL", "Enter -  initOdodmetryEncoders");

        verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RobotLog.ii("CAL", "Exit -  initOdodmetryEncoders");
    }

    /*public void initMotorEncoders() {
        RobotLog.ii("CAL", "Enter -  initMotorEncoders");

        // Sets the mode of the motors to run WITH encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.ii("CAL", "Exit -  initMotorEncoders");
    }
*/


    public void moveHolonomic(double x, double y , double z)
    {
        double max_power = 0.6;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x - z, min_power, max_power);
        double fr_power = Range.clip(y - x + z, min_power, max_power);
        double br_power = Range.clip(y + x + z, min_power, max_power);
        double bl_power = Range.clip(y - x - z, min_power, max_power);
        RobotLog.ii("CAL", "moveHolonomic - Enter x(%f), y(%f), z(%f)", x, y, z);
        RobotLog.ii("CAL", "moveHolonomic - Enter fl(%f), fr(%f), bl(%f), br(%f)", fl_power,fr_power, bl_power, br_power );

        // Sets the power of the motors to the power defined above

        setPowerAll(fl_power, fr_power, bl_power, br_power);
        RobotLog.ii("CAL", "moveHolonomic - Exit ");

    }

    public void setPowerAll(double fl_power, double fr_power, double bl_power, double br_power){
        leftMotor.setPower(fl_power);
        rightMotor.setPower(fr_power);
        backleftMotor.setPower(bl_power);
        backrightMotor.setPower(br_power);


    }
    //extra motions to move slowly go in case we are in a situation like that
    public void forwardSlow() {
        leftMotor.setPower(Range.clip(leftMotor.getPower() + 0.01, 0.3, 1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() + 0.01, 0.3, 1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() + 0.01, 0.3, 1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() + 0.01, 0.3, 1.0));
    }

    public void backwardSlow() {
        leftMotor.setPower(Range.clip(leftMotor.getPower() - 0.01, -0.3, -1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() - 0.01, -0.3, -1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() - 0.01, -0.3, -1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() - 0.01, -0.3, -1.0));
    }

}
