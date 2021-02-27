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
    public BNO055IMU imu = null;
    DcMotorEx verticalLeft, verticalRight;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        verticalLeft = ahwMap.get(DcMotorEx.class, "M1");
        verticalRight = ahwMap.get(DcMotorEx.class, "M2");
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

        RobotLog.ii("CAL", "Exit - init");

    }

}
