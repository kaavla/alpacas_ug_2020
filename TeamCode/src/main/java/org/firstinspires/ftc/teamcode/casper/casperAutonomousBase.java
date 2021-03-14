package org.firstinspires.ftc.teamcode.casper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class casperAutonomousBase extends LinearOpMode {

    public enum wobbleGoalMode {
        WOBBLE_GOAL_UP,
        WOBBLE_GOAL_DOWN,
        WOBBLE_GOAL_CLAW_OPEN,
        WOBBLE_GOAL_CLAW_CLOSE
    }
    static final double PULLEY_COUNTS_PER_INCH = (50.9 * 28) / (1 * 3.1415); //gobilda 5202 117 rpm motors
    static final double SHOOT_MOTOR_POWER = 0.4; //gobilda 5202 117 rpm motors

    public boolean targetVisible = false;

    public casperMecanumDrive robot;
    public ElapsedTime runtime = new ElapsedTime();

    public VectorF translation;
    public Orientation rotation;

    public void logTiming(String S, ElapsedTime T)
    {
        RobotLog.ii(S, "Timing  = %.03f", T.milliseconds());
        T.reset();

    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Empty Function
    }

    public boolean getVuforiaRefPos(double timeoutmS)
    {
        runtime.reset();
        while (opModeIsActive() && !isStopRequested() && (runtime.milliseconds() < timeoutmS)) {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : robot.allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        robot.lastLocation = robotLocationTransform;
                        // express position (translation) of robot in inches.
                        translation = robot.lastLocation.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / robot.mmPerInch, translation.get(1) / robot.mmPerInch, translation.get(2) / robot.mmPerInch);

                        // express the rotation of the robot in degrees.
                        rotation = Orientation.getOrientation(robot.lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        telemetry.update();
                    }
                    return true;
                } else {
                    telemetry.addData("Visible Target", "none");
                    telemetry.update();
                }
            }
        }
        return false;
    }

    public void operateWobbleClaw (wobbleGoalMode m) {
        if (m == wobbleGoalMode.WOBBLE_GOAL_CLAW_OPEN) {
            robot.openWobbleClaw();
            sleep(400);
        }else{
            robot.closeWobbleClaw();
            sleep(1000);

        }

    }

    public void moveWobbleGoal (wobbleGoalMode m) {
        int newTargetPosition = 0;

        double speed   = 0.3;  //Speed with which to move the wobble goal
        double Inches   = 1;   //Inches to move the goal
        double timeoutS = 3;   //Timeout

        //Reset the encoder
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        if (m == wobbleGoalMode.WOBBLE_GOAL_UP) {
            //Move Wobble Goal UP
            newTargetPosition = robot.wobbleMotor.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);

        } else if (m == wobbleGoalMode.WOBBLE_GOAL_DOWN) {
            //Move Wobble Goal DOWN
            newTargetPosition = robot.wobbleMotor.getCurrentPosition() + (int) (-1 * Inches * PULLEY_COUNTS_PER_INCH);
        } else {
            Inches = 0;
            newTargetPosition = robot.wobbleMotor.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);
        }
        robot.wobbleMotor.setTargetPosition(newTargetPosition);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleMotor.setPower(Math.abs(speed));

        runtime.reset();
        while (opModeIsActive() && !isStopRequested() &&
                (runtime.seconds() < timeoutS) &&
                (robot.wobbleMotor.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.wobbleMotor.getCurrentPosition(),
                    robot.wobbleMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.wobbleMotor.setPower(0);
        sleep(200);

    }

    public int getNumRings(double timeoutmS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < timeoutmS))
        {
            //RobotLog.ii("CASPER", "enter - getnumrings");
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // step through the list of recognitions and display boundary info.
                    String strLabel;
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        RobotLog.ii("CASPER", "size = %d", updatedRecognitions.size());
                        strLabel = recognition.getLabel();
                        telemetry.addData(String.format("label %d", i), strLabel);

                        telemetry.addData(String.format("  left,top %d", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom %d", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (strLabel.equals(robot.LABEL_FIRST_ELEMENT)) {
                            RobotLog.ii("CASPER", "4 rings");
                            telemetry.addData("numRings = 4", "ddd");
                            telemetry.update();
                            return 4;
                        }
                        if (strLabel.equals(robot.LABEL_SECOND_ELEMENT)) {
                            telemetry.addData("numRings = 1", "ddd");
                            telemetry.update();
                            RobotLog.ii("CASPER", "1 rings");
                            return 1;
                        }
                        telemetry.update();
                    }
                }
            }
            sleep(50);
        }
        telemetry.addData("numRings = 0", "ddd");
        telemetry.update();

        return 0;
    }
    public void openPushRing() {robot.openPushRing();}
    public void closePushRing() {robot.closePushRing();}
    public void startShootMotor() {
        robot.shootMotorLeft.setPower(SHOOT_MOTOR_POWER);
    }
    public void stopShootMotor() {
        robot.shootMotorLeft.setPower(0.0);
    }

    public void shooterMotorAuto() {
        int newTargetPosition = 0;

        double speed   = 0.8;  //Speed with which to move the wobble goal
        double Inches   = 10;   //Inches to move the goal
        double timeoutS = 3;   //Timeout

        //Reset the encoder
        robot.shootMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {
                newTargetPosition = robot.shootMotorLeft.getCurrentPosition() + (int) (Inches * PULLEY_COUNTS_PER_INCH);

            robot.shootMotorLeft.setTargetPosition(newTargetPosition);
            robot.shootMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.shootMotorLeft.setPower(Math.abs(speed));

            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.shootMotorLeft.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.shootMotorLeft.getCurrentPosition(),
                        robot.shootMotorLeft.getCurrentPosition());
                telemetry.update();
            }
        }

        // Stop all motion;
        robot.shootMotorLeft.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.shootMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


}

