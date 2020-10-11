package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class JARVISHW
{
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor backrightMotor = null;
    public DcMotor backleftMotor = null;

    public DcMotor slide_1 = null;
    //public DcMotor slide_2 = null;
    public DcMotor slide_3 = null;
    public Servo capServo = null;
    public Servo clawServo = null;

    public Servo FLServo = null;
    public Servo FRServo = null;


    public CRServo  tapeServo = null;
    //public Servo CollectRightServo = null;
    public DcMotor CollectLeftMotor = null;
    public DcMotor CollectRightMotor = null;

    public Servo GrabberLeftTurnServo = null;
    public Servo GrabberLeftClawServo = null;
    public Servo GrabberRightTurnServo = null;
    public Servo GrabberRightClawServo = null;


    //public CRServo VexServo = null;

    public DistanceSensor sensorDistanceRight = null;
    public DistanceSensor sensorDistanceLeft = null;
    public DistanceSensor sensorDistanceFL = null;
    // public ColorSensor sensorColor = null;
    public ColorSensor sensorColorLeft = null;
    public ColorSensor sensorColorRight = null;
    public TouchSensor touch_sensor = null;

    public BNO055IMU imu = null;

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


        slide_1  = ahwMap.get(DcMotor.class, "slide_1");
        //slide_2  = ahwMap.get(DcMotor.class, "slide_2");
        slide_3 = ahwMap.get(DcMotor.class, "slide_3");

        capServo = ahwMap.get(Servo.class, "capServo");
        clawServo = ahwMap.get(Servo.class, "clawServo");


        FLServo = ahwMap.get(Servo.class, "FLServo");
        FRServo = ahwMap.get(Servo.class, "FRServo");

        tapeServo = ahwMap.get(CRServo.class, "tapeServo");
        //CollectRightServo = ahwMap.get(Servo.class, "CollectRightServo");
        //VexServo = ahwMap.get(CRServo.class, "vex");

        CollectLeftMotor = ahwMap.get(DcMotor.class, "CollectLeftMotor");
        CollectRightMotor = ahwMap.get(DcMotor.class, "CollectRightMotor");

        GrabberLeftTurnServo = ahwMap.get(Servo.class, "GrabberLeftTurnServo");
        GrabberLeftClawServo = ahwMap.get(Servo.class, "GrabberLeftClawServo");

        GrabberRightTurnServo = ahwMap.get(Servo.class, "GrabberRightTurnServo");
        GrabberRightClawServo = ahwMap.get(Servo.class, "GrabberRightClawServo");

        sensorColorLeft = ahwMap.get(ColorSensor.class, "sensor_color_left");
        sensorColorRight = ahwMap.get(ColorSensor.class, "sensor_color_right");

        sensorDistanceLeft = ahwMap.get(DistanceSensor.class, "sensorDistanceLeft");
        sensorDistanceRight = ahwMap.get(DistanceSensor.class, "sensorDistanceRight");
        sensorDistanceFL = ahwMap.get(DistanceSensor.class, "sensorDistanceFL");

        touch_sensor = ahwMap.get(TouchSensor.class, "touch_sensor");

        //markerServo = ahwMap.get(Servo.class, "MServo");

        imu = ahwMap.get(BNO055IMU.class, "imu 1");

        //initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        //Invert direction for left motors
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //slide_1.setDirection(DcMotorSimple.Direction.REVERSE);
        //slide_2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_3.setDirection(DcMotorSimple.Direction.REVERSE);
        FLServo.setDirection(Servo.Direction.REVERSE);
        GrabberRightTurnServo.setDirection(Servo.Direction.REVERSE);
        GrabberRightClawServo.setDirection(Servo.Direction.REVERSE);
        tapeServo.setDirection(CRServo.Direction.REVERSE);
        CollectRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        stopAllMotors();

        //Set zero power behavior to braking
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateClawInline();
        openClaw();

        setGrabberUp(0);
        setGrabberUp(1);

        closeGrabberClaw(0);
        closeGrabberClaw(1);

        RobotLog.ii("CAL", "Exit - init");

    }

    //resets the power to zero before starting the action
    public void stopAllMotors() {
        RobotLog.ii("CAL", "Stopping All motors");
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);
        slide_1.setPower(0);
        //slide_2.setPower(0);
        slide_3.setPower(0);
        measureTapeStop();
    }

    public void initMotorNoEncoders() {
        RobotLog.ii("CAL", "Enter -  initMotorNoEncoders");

        // Sets the mode of the motors to run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //slide_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RobotLog.ii("CAL", "Exit -  initMotorNoEncoders");
    }


    public void initMotorEncoders() {
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

        slide_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slide_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        RobotLog.ii("CAL", "Exit -  initMotorEncoders");
    }



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
        leftMotor.setPower(fl_power);
        rightMotor.setPower(fr_power);
        backleftMotor.setPower(bl_power);
        backrightMotor.setPower(br_power);

        RobotLog.ii("CAL", "moveHolonomic - Exit ");

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

    // All of the functions that move the motors and servos are below
    public void slidesUp(double power)
    {
        slide_1.setPower(power);
        //slide_2.setPower(power);
    }

    public void slidesDown(double power)
    {
        slidesUp(-1*power);
    }

    public void slideOut(double power)
    {
        slide_3.setPower(power);
    }

    public void slideIn(double power)
    {
        slideOut(-1*power);
    }

    public void rotateClawPerpendicular()
    {
        capServo.setPosition(0.05);
    }

    public void rotateClawInline()
    {
        //capServo.setPosition(0.25);
        capServo.setPosition(0.4);
    }

    public void openClaw()
    {
        clawServo.setPosition(0.55);
    }
    public void closeClaw()
    {
        clawServo.setPosition(0.8);
    }

    public void closeClawSensor(double timeoutS)
    {
        if (timeoutS < 10)
        clawServo.setPosition(0.8);
    }

    public void moveFoundationServoUp() {
        FLServo.setPosition(0);
        FRServo.setPosition(0);
    }

    public void moveFoundationServoDown() {
        FLServo.setPosition(0.5);
        FRServo.setPosition(0.5);
    }

    public void closeCapStoneClaw()
    {
        capServo.setPosition(0.0);
    }

    public void openCapStoneClaw()
    {
        //capServo.setPosition(0.25);
        capServo.setPosition(0.7);
    }

    public void measureTapeOut() {
        tapeServo.setPower(1);
        //VexServo.setPower(1);
    }

    public void measureTapeIn() {
        tapeServo.setPower(-1);
        //VexServo.setPower(1);
    }

    public void measureTapeStop() {
        tapeServo.setPower(0);
        //VexServo.setPower(1);
    }


    public void resetCollectionServo () {
        CollectLeftMotor.setPower(0);
        CollectRightMotor.setPower(0);

    }

    public void setCollectionServo() {
        CollectLeftMotor.setPower(1);
        CollectRightMotor.setPower(1);
    }

    public void setGrabberDown(int side) {
        if (side == 0) {
            //Left
            GrabberLeftTurnServo.setPosition(0.45);
        } else
        {
           //Right
           GrabberRightTurnServo.setPosition(0.45);
       }

    }

    public void setGrabberHalfDown(int side) {
        if (side == 0) {
            //Left
            GrabberLeftTurnServo.setPosition(0.4);
        } else
        {
            //Right
            GrabberRightTurnServo.setPosition(0.37);
        }
    }


    public void setGrabberUp(int side) {
        if (side == 0) {
            //Left
            GrabberLeftTurnServo.setPosition(0.1);
        } else
        {
            //Right
            GrabberRightTurnServo.setPosition(0.1);
        }
    }

    public void closeGrabberClaw(int side) {

        if (side == 0) {
            //Left
            GrabberLeftClawServo.setPosition(0);

        } else
        {
            //Right

            GrabberRightClawServo.setPosition(0);
        }
    }

    public void openGrabberClaw(int side) {

        if (side == 0) {
            //Left
            GrabberLeftClawServo.setPosition(0.5);
        } else
        {
            //Right

            GrabberRightClawServo.setPosition(0.5);
        }
    }



}