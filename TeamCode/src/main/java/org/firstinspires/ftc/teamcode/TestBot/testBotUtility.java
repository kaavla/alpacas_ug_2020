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

    public testBotUtility.Direction direction;

    public double distance_traveled = 0;

    static final double COUNTS_PER_MOTOR_REV = 145.6;    // eg: goBilda 5202 Motor Encoder 5.2*28
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double PULLEY_COUNTS_PER_INCH = (50.9 * 28) / (1 * 3.1415); //gobilda 5202 117 rpm motors
    static final double INOUT_COUNTS_PER_INCH = (19.2 * 28) / (2 * 3.1415); //gobilda 5202 117 rpm motors

    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.7;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    final String VUFORIA_KEY = "ATVrdOT/////AAABmegFa9L6UUB2ljwRjEStPmU7NS6gi/+GLAe6uAv7o+cB7+pj9EORNLk32cxovTaRj+rUeNw75EMjs5jM0K2OlNn8iO861FyZ5bqnHeBQRr/tR4NIZkQq4ak2zpPLQyyGFzhEkHjnhenYh0dyvxluXF79u8VwJ+g77slCyrCjvgMp6VfEAPLpVJmjzq4hRJMtjYpoRp/agnYFU8HVnmQeGRbjKi1PHLbhP98IkGMowt6Hlobdd2l0vt7msVhwNombHz0XcwJEjwnRKoOkeg7s+kIWvd5paYiO/bnClo9DahFboEFWw1/9wutXgI6/7AGcvwZzkk1HwRh3qZRAWNUSq1hrcjdq9f2QXAYyiqd3wLpT";


    public TFObjectDetector tfod = null;
    public VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode() {
        //Empty Function
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
        // restart imu movement tracking.
        resetAngle();


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


    public void myEncoderDrive(JARVISAutonomousBase.Direction direction, double speed, double Inches, double timeoutS, JARVISAutonomousBase.SensorsToUse sensors_2_use) {
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
            if (direction == JARVISAutonomousBase.Direction.FORWARD) {
                //Go forward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == JARVISAutonomousBase.Direction.BACKWARD) {
                //Go backward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == JARVISAutonomousBase.Direction.STRAFE_RIGHT) {
                //Strafe Right
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == JARVISAutonomousBase.Direction.STRAFE_LEFT) {
                //Strafe Left
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);

            } else if (direction == JARVISAutonomousBase.Direction.DIAGONAL_LEFT) {
                //Left Diagonal
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == JARVISAutonomousBase.Direction.DIAGONAL_RIGHT) {
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


            }
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