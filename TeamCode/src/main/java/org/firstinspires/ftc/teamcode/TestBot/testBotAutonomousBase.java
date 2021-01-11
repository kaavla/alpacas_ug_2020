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
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double PULLEY_COUNTS_PER_INCH = (50.9 * 28) / (1 * 3.1415); //gobilda 5202 117 rpm motors
    static final double INOUT_COUNTS_PER_INCH = (19.2 * 28) / (2 * 3.1415); //gobilda 5202 117 rpm motors
    public static double WHEEL_RADIUS = 1.9685; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 17.66; // in
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
        //HELLO PLEASE ADD SOMETHING TO INITIALIZE CLOSING THE WOBBLE GOAL CLAW
        //THANKS
        //robot.initMotorEncoders();
        /*
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }
        */
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


    public void rotate(int degrees, double power) {
        //logs that get added to a file to see what was wrong with the robot and the sequences of it
        RobotLog.ii("CAL", "Enter - rotate - degrees=%d, power=%f",
                degrees, power);
        robot.initMotorNoEncoders();
        // restart imu movement tracking.
        resetAngle();

        if (degrees < 0) {   // turn right.
            robot.moveHolonomic(0, 0, power * -1);
        } else if (degrees > 0) {   // turn left.
            robot.moveHolonomic(0, 0, power * 1);
        } else return;


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && !isStopRequested() && getAngle() == 0) {
            }

            while (opModeIsActive() && !isStopRequested() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && !isStopRequested() && getAngle() < degrees) {
            }

        // turn the motors off.
        power = 0;
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
        robot.backleftMotor.setPower(power);
        robot.backrightMotor.setPower(power);

        // wait for rotation to stop.
        sleep(50);

        // reset angle tracking on new heading.
        resetAngle();
        RobotLog.ii("CAL", "Exit - rotate");
    }

    public void rotateFrontUsingOneSide(int degrees, double speed) {
        //logs that get added to a file to see what was wrong with the robot and the sequences of it
        RobotLog.ii("CAL", "Enter - rotate - degrees=%d, power=%f",
                degrees, speed);

        // restart imu movement tracking.
        resetAngle();

        if (degrees < 0) {   // turn right.
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(1 * speed);
            robot.backleftMotor.setPower(0);
            robot.backrightMotor.setPower(1 * speed);
        } else if (degrees > 0) {   // turn left.
            robot.leftMotor.setPower(1 * speed);
            robot.rightMotor.setPower(0);
            robot.backleftMotor.setPower(1 * speed);
            robot.backrightMotor.setPower(0);
        } else return;


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && !isStopRequested() && getAngle() == 0) {
            }

            while (opModeIsActive() && !isStopRequested() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && !isStopRequested() && getAngle() < degrees) {
            }

        // turn the motors off.
        int power = 0;
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
        robot.backleftMotor.setPower(power);
        robot.backrightMotor.setPower(power);

        // wait for rotation to stop.
        sleep(50);

        // reset angle tracking on new heading.
        resetAngle();
        RobotLog.ii("CAL", "Exit - rotate");
    }

    public void rotateUsingOneSide(int degrees, double speed) {
        //logs that get added to a file to see what was wrong with the robot and the sequences of it
        RobotLog.ii("CAL", "Enter - rotate - degrees=%d, power=%f",
                degrees, speed);

        // restart imu movement tracking.
        resetAngle();

        if (degrees < 0) {   // turn right.
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(-1 * speed);
            robot.backleftMotor.setPower(0);
            robot.backrightMotor.setPower(-1 * speed);
        } else if (degrees > 0) {   // turn left.
            robot.leftMotor.setPower(-1 * speed);
            robot.rightMotor.setPower(0);
            robot.backleftMotor.setPower(-1 * speed);
            robot.backrightMotor.setPower(0);
        } else return;


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && !isStopRequested() && getAngle() == 0) {
            }

            while (opModeIsActive() && !isStopRequested() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && !isStopRequested() && getAngle() < degrees) {
            }

        // turn the motors off.
        int power = 0;
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
        robot.backleftMotor.setPower(power);
        robot.backrightMotor.setPower(power);

        // wait for rotation to stop.
        sleep(50);

        // reset angle tracking on new heading.
        resetAngle();
        RobotLog.ii("CAL", "Exit - rotate");
    }


    public void myEncoderDrive(Direction direction, double speed, double Inches, double timeoutS, SensorsToUse sensors_2_use) {
        int newLeftTarget = 0;
        int newRightTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset the encoder
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.FORWARD) {
                //Go forward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.BACKWARD) {
                //Go backward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.STRAFE_RIGHT) {
                //Strafe Right
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.STRAFE_LEFT) {
                //Strafe Left
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.DIAGONAL_LEFT) {
                //Left Diagonal
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.DIAGONAL_RIGHT) {
                //Right Diagonal
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else {
                Inches = 0;
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }


            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);
            robot.backleftMotor.setTargetPosition(newLeftBackTarget);
            robot.backrightMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            robot.backleftMotor.setPower(Math.abs(speed));
            robot.backrightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy())) {
/*
                if (sensors_2_use == SensorsToUse.USE_DISTANCE_LEFT) {
                    if(robot.sensorDistanceLeft.getDistance(DistanceUnit.INCH) <= 7) {
                        robot.stopAllMotors();

                        telemetry.addData("LeftDistSensor", "The robot is %7f inches from crashing.",
                                robot.sensorDistanceLeft.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                        break;

                    }
                }

                if (sensors_2_use == SensorsToUse.USE_DISTANCE_RIGHT) {
                    if(robot.sensorDistanceRight.getDistance(DistanceUnit.INCH) <= 7) {
                        robot.stopAllMotors();

                        telemetry.addData("RightDistSensor", "The robot is %7f inches from crashing.",
                                robot.sensorDistanceRight.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                        break;

                    }
                }
                if (sensors_2_use == SensorsToUse.USE_COLOR_LEFT) {

                    if (myDetectSkystone(SideToUse.USE_LEFT, 2) == true)
                    {
                        robot.stopAllMotors();
                        //telemetry.addData("LeftColorSensor", "The robot detected Skystone");
                        //telemetry.update();
                        break;
                    }
                }
                if (sensors_2_use == SensorsToUse.USE_COLOR_RIGHT) {

                    if (myDetectSkystone(SideToUse.USE_RIGHT, 2) == true)
                    {
                        robot.stopAllMotors();
                        //telemetry.addData("LeftColorSensor", "The robot detected Skystone");
                        //telemetry.update();
                        break;
                    }
                }
                if (sensors_2_use == SensorsToUse.USE_DISTANCE_RIGHT_BLD) {
                    if(robot.sensorDistanceRight.getDistance(DistanceUnit.INCH) <= 2) {
                        robot.stopAllMotors();

                        telemetry.addData("RightDistSensor", "The robot is %7f inches from crashing.",
                                robot.sensorDistanceRight.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                        break;
                    }
                }

                if (sensors_2_use == SensorsToUse.USE_DISTANCE_LEFT_BLD) {
                    if(robot.sensorDistanceLeft.getDistance(DistanceUnit.INCH) <= 2) {
                        robot.stopAllMotors();

                        telemetry.addData("LeftDistSensor", "The robot is %7f inches from crashing.",
                                robot.sensorDistanceLeft.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                        break;
                    }
                }

                if (sensors_2_use == SensorsToUse.USE_DISTANCE_LEFT_FDT) {
                    if(robot.sensorDistanceLeft.getDistance(DistanceUnit.INCH) > 20) {
                        robot.stopAllMotors();
                    }
                }

                if (sensors_2_use == SensorsToUse.USE_DISTANCE_RIGHT_FDT) {
                    if(robot.sensorDistanceRight.getDistance(DistanceUnit.INCH) > 20) {
                        robot.stopAllMotors();
                    }
                }

                if (sensors_2_use == SensorsToUse.USE_DISTANCE_FRONT) {
                    if(robot.sensorDistanceFL.getDistance(DistanceUnit.INCH) <23) {
                        robot.stopAllMotors();
                    }
                }


            }

 */
        }



                // Stop all motion;
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                robot.backleftMotor.setPower(0);
                robot.backrightMotor.setPower(0);

                distance_traveled = robot.rightMotor.getCurrentPosition() / COUNTS_PER_INCH;

                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                sleep(50);   // optional pause after each move
            }
        }




    /*
    public void myEncoderSlide(Direction direction, double speed, double Inches, double timeoutS, SensorsToUse sensors_2_use) {
        int newLeftTarget = 0;
        int newRightTarget = 0;
        RobotLog.ii("CAL", "Enter - myEncoderSlide -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);

        robot.slide_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slide_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.slide_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.slide_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Reset the encoder

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.SLIDE_UP) {
                //Go forward
                newLeftTarget = robot.slide_1.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                newRightTarget = robot.slide_2.getCurrentPosition() + (int) (1*Inches * PULLEY_COUNTS_PER_INCH);

            } else if (direction == Direction.SLIDE_DOWN) {
                //Go backward
                newLeftTarget = robot.slide_1.getCurrentPosition() + (int) (-1 * Inches * PULLEY_COUNTS_PER_INCH);
                newRightTarget = robot.slide_2.getCurrentPosition() + (int) (-1* Inches * PULLEY_COUNTS_PER_INCH);
            } else {
                Inches = 0;
                newLeftTarget = robot.slide_1.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                newRightTarget = robot.slide_2.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
            }


            robot.slide_1.setTargetPosition(newLeftTarget);
            robot.slide_2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.slide_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.slide_1.setPower(Math.abs(speed));
            robot.slide_2.setPower(Math.abs(speed));

            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.slide_1.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.slide_1.getCurrentPosition(),
                        robot.slide_2.getCurrentPosition());
                telemetry.update();
            }
        }

        // Stop all motion;
        robot.slide_1.setPower(0);
        robot.slide_2.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.slide_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slide_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
*/
            public void myWobbleGoal (Direction direction, double speed, double Inches, double timeoutS, SensorsToUse sensors_2_use) {
                int newLeftTarget = 0;
                int newRightTarget = 0;
                RobotLog.ii("CAL", "Enter - myEncoderSlide -  speed=%f, Inches=%f, timeout=%f",
                        speed, Inches, timeoutS);

                //Reset the encoder
                robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //robot.slide_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Ensure that the op mode is still active
                if (opModeIsActive() && !isStopRequested()) {

                    // Determine new target position, and pass to motor controller
                    if (direction == Direction.SLIDE_UP) {
                        //Go forward
                        newLeftTarget = robot.wobbleMotor.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                        //newRightTarget = robot.slide_2.getCurrentPosition() + (int) (1*Inches * PULLEY_COUNTS_PER_INCH);

                    } else if (direction == Direction.SLIDE_DOWN) {
                        //Go backward
                        newLeftTarget = robot.wobbleMotor.getCurrentPosition() + (int) (-1 * Inches * PULLEY_COUNTS_PER_INCH);
                        //newRightTarget = robot.slide_2.getCurrentPosition() + (int) (-1* Inches * PULLEY_COUNTS_PER_INCH);
                    } else {
                        Inches = 0;
                        newLeftTarget = robot.wobbleMotor.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                        //newRightTarget = robot.slide_2.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                    }


                    robot.wobbleMotor.setTargetPosition(newLeftTarget);
                    //robot.slide_2.setTargetPosition(newRightTarget);

                    // Turn On RUN_TO_POSITION
                    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //robot.slide_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // reset the timeout time and start motion.
                    runtime.reset();

                    robot.wobbleMotor.setPower(Math.abs(speed));
                    if (direction == Direction.SLIDE_DOWN) {
                        //robot.slide_2.setPower((-1*speed));
                    } else {
                        //robot.slide_2.setPower((1*speed));

                    }

                    while (opModeIsActive() && !isStopRequested() &&
                            (runtime.seconds() < timeoutS) &&
                            (robot.wobbleMotor.isBusy())) {


                        // Display it for the driver.
                        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d",
                                robot.wobbleMotor.getCurrentPosition(),
                                robot.wobbleMotor.getCurrentPosition());
                        telemetry.update();
                    }
                }

                // Stop all motion;
                robot.wobbleMotor.setPower(0);
                //robot.slide_2.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //robot.slide_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
    /*
        public void myEncoderSlide2(Direction direction, double speed, double Inches, double timeoutS, SensorsToUse sensors_2_use) {
            int newLeftTarget = 0;
            int newRightTarget = 0;
            RobotLog.ii("CAL", "Enter - myEncoderSlide -  speed=%f, Inches=%f, timeout=%f",
                    speed, Inches, timeoutS);

            //Reset the encoder
            robot.slide_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //robot.slide_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.slide_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Ensure that the op mode is still active
            if (opModeIsActive() && !isStopRequested()) {

                // Determine new target position, and pass to motor controller
                if (direction == Direction.SLIDE_UP) {
                    //Go forward
                    //newLeftTarget = robot.slide_1.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                    newRightTarget = robot.slide_2.getCurrentPosition() + (int) (1*Inches * PULLEY_COUNTS_PER_INCH);

                } else if (direction == Direction.SLIDE_DOWN) {
                    //Go backward
                    //newLeftTarget = robot.slide_1.getCurrentPosition() + (int) (-1 * Inches * PULLEY_COUNTS_PER_INCH);
                    newRightTarget = robot.slide_2.getCurrentPosition() + (int) (-1* Inches * PULLEY_COUNTS_PER_INCH);
                } else {
                    Inches = 0;
                    //newLeftTarget = robot.slide_1.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                    newRightTarget = robot.slide_2.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
                }


                //robot.slide_1.setTargetPosition(newLeftTarget);
                robot.slide_2.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                //robot.slide_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slide_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                robot.slide_2.setPower(Math.abs(speed));

                while (opModeIsActive() && !isStopRequested() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.slide_2.isBusy())) {


                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.slide_1.getCurrentPosition(),
                            robot.slide_2.getCurrentPosition());
                    telemetry.update();
                }
            }

            // Stop all motion;
            //robot.slide_1.setPower(0);
            robot.slide_2.setPower(0);

            // Turn off RUN_TO_POSITION
            //robot.slide_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.slide_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    public void myEncoderInOutSlide(Direction direction, double speed, double Inches, double timeoutS, SensorsToUse sensors_2_use) {
        int newLeftTarget = 0;
        RobotLog.ii("CAL", "Enter - myEncoderInOutSlide -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);

        robot.slide_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slide_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.SLIDE_IN) {
                //Go forward
                newLeftTarget = robot.slide_3.getCurrentPosition() + (int) (Inches * INOUT_COUNTS_PER_INCH);

            } else if (direction == Direction.SLIDE_OUT) {
                //Go backward
                newLeftTarget = robot.slide_3.getCurrentPosition() + (int) (-1 * Inches * INOUT_COUNTS_PER_INCH);
            } else {
                Inches = 0;
                newLeftTarget = robot.slide_3.getCurrentPosition() + (int) (Inches * INOUT_COUNTS_PER_INCH);
            }


            robot.slide_3.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.slide_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.slide_3.setPower(Math.abs(speed));

            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.slide_3.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newLeftTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.slide_3.getCurrentPosition(),
                        robot.slide_3.getCurrentPosition());
                telemetry.update();
            }
        }

        // Stop all motion;
        robot.slide_3.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.slide_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

     */
//SONIA AND REESA PLEASE ADD YOUR TENSORFLOW STUFF HERE
            /*
    public void myTFOD(double timeoutS) {
        {
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.

            boolean strafeDone = false;
            RobotLog.ii("CAL", "myTFOD - Enter");

            while (opModeIsActive() && !isStopRequested()) {

                if (tfod == null) {
                    robot.moveHolonomic(0, 0, 0);
                    break;
                }

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    if (updatedRecognitions.size() == 0) {
                        robot.moveHolonomic(0, 0, 0);
                    } else {
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                double targetHeightRatio = (float) 0.8;
                                double imageHeight = recognition.getImageHeight();
                                double imageWidth = recognition.getImageWidth();
                                double objectHeight = recognition.getHeight();
                                double objectHeightRatio = objectHeight / imageHeight;
                                double power = 0.1;
                                double mid = (recognition.getLeft() + recognition.getRight()) / 2;
                                double i_left = recognition.getLeft();
                                double i_right = recognition.getRight();

                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(" ", "Image Width (%.1f), image Height (%.1f), object Height (%.1f)",
                                        imageWidth, imageHeight, objectHeight);
                                telemetry.addData(String.format(" init left,right (%d)", i), "%.03f , %.03f",
                                        i_left, i_right);

                                if (strafeDone == false) {
                                    if (mid < (640 - 100)) {
                                        telemetry.addData(String.format(" mid(%f) < 540 ", mid), "");
                                        robot.moveHolonomic(-1 * power, 0, 0);
                                    } else if (mid > (640 + 100)) {
                                        telemetry.addData(String.format(" mid(%f) > 740 ", mid), "");
                                        robot.moveHolonomic(1 * power, 0, 0);
                                    } else {
                                        strafeDone = true;
                                        robot.moveHolonomic(0, 0, 0);
                                    }
                                }

                                if (strafeDone == true) {
                                    telemetry.addData(" ", " Shank Strafe done");

                                    if (objectHeightRatio < targetHeightRatio) {
                                        telemetry.addData(" ", " SHANK object < target power=%f", power);

                                        robot.moveHolonomic(0, 1 * power, 0);
                                    } else {
                                        robot.moveHolonomic(0, 0, 0);
                                    }
                                }

                            } else {
                                telemetry.addData("Not a skystone", " ");

                            }
                        }
                    }
                    telemetry.update();
                }

                //RobotLog.ii("CAL", "while opModeIsActive and !isStopRequested - Enter");
            }
            RobotLog.ii("CAL", "Exit if opModeIsActive");

            if (tfod != null) {
                tfod.shutdown();
            }
            RobotLog.ii("CAL", "myTFOD - Exits");

        }
    }

             */
            //SONIA AND REESA PLEASE DELETE THIS IF YOU DON'T NEED IT
            /*
    public boolean myTFOD2(double timeoutS) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        boolean strafeDone = false;
        RobotLog.ii("CAL", "myTFOD - Enter");


        while (opModeIsActive() && !isStopRequested()) {

            if (tfod == null) {
                robot.moveHolonomic(0, 0, 0);
                break;
            }

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                if (updatedRecognitions.size() == 0) {
                    robot.moveHolonomic(0, 0, 0);
                } else {
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                }
                telemetry.update();
                RobotLog.ii("CAL", "Exit if opModeIsActive");

                if (tfod != null) {
                    tfod.shutdown();
                }
                RobotLog.ii("CAL", "myTFOD - Exits");

            }
        }
        return false;
    }

    public void moveFoundationServoDown () {
        // Checks if the servos are = null or not because that is what causes the
        // "null pointer exception". Once it is checked, the servos will run.
        robot.moveFoundationServoDown();
        /*
        if (robot.FLServo != null) {
            robot.FLServo.setPosition(0.21);
        }
        if (robot.FRServo != null) {
            robot.FRServo.setPosition(0.21);
        }
         */
        }
    
