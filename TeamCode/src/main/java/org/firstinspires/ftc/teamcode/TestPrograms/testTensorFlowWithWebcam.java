/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.JARVISHW;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "testTensorFlowWithWebcam", group = "Callisto")
@Disabled
public class testTensorFlowWithWebcam extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AXSRCQP/////AAABmZeKs8E+bExYlViUoU4W3x9D+ZqA3HLfy3PxlWiz0h5wh/awa/Oe9lra0C0CqlyRducvIyV5egl7zTYvGsbA34h3hCAV1eQtpnzQtYulVYRxD6W2Lnl47omLOHjXv3fTXLPnPDBugwDQUCqw4tN58FFEN5xoKEIPWwaQuOg43WHpfa6wenMv+bxuiwxM0Ciy+2gad/kkc+MTWzsFAL/yjTQhq718BNLr1FYZveMEwFHS43kILSKaL/+3/YGqd677av/z5tVDLkSRPUDuEYKIB1P0uCJd5AhIPnVvNigICEUxETMZiEt0RmKoQ3x9S6Y8AelTJgpHeuVgDHy5BmNP877er8Bsqr+WfHGho64CNbUx\n";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //   eg: a Positive RelativeBearing means the robot must turn CCW to point at the target image.
    private JARVISHW robotJARVIS = new JARVISHW();

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        boolean strafeDone = false;
        robotJARVIS.init(hardwareMap);

        initVuforia();
        /**
         * TODO - needs revisiting
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
         */

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        if (updatedRecognitions.size() == 0) {
                            robotJARVIS.moveHolonomic(0, 0, 0);
                        }
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                                double targetHeightRatio = (float) 0.8;
                                double imageHeight = recognition.getImageHeight();
                                double imageWidth = recognition.getImageWidth();
                                double objectHeight = recognition.getHeight();
                                double objectHeightRatio = objectHeight / imageHeight;

                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(" ", "Image Width (%.1f), image Height (%.1f), object Height (%.1f)",
                                        imageWidth, imageHeight, objectHeight);
                                telemetry.addData(String.format("  left,right (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getRight());

                                double obj_h_mm = objectHeight * 25.4 / 424.0;
                                double f2 = (60 * 5.5 * 25.4 / obj_h_mm) + 60;
                                telemetry.addData(" ", " Distance = %.1f, inch", f2 / 25.4);
                                double power = 0.3;
                            /*
                            if (recognition.getLeft() > (680 - 100)) {
                                robotJARVIS.moveHolonomic(power, 0, 0);
                            } else {
                                strafeDone = true;
                                robotJARVIS.moveHolonomic(0, 0, 0);
                            }*/

                                double mid = (recognition.getLeft() + recognition.getRight()) / 2;
                                if (mid < (640 - 100)) {
                                    robotJARVIS.moveHolonomic(-1 * power, 0, 0);
                                } else if (mid > (640 + 100)) {
                                    robotJARVIS.moveHolonomic(power, 0, 0);
                                } else {
                                    strafeDone = true;
                                    robotJARVIS.moveHolonomic(0, 0, 0);
                                }

                                if (strafeDone == true) {

                                    if (objectHeightRatio < targetHeightRatio) {
                                        robotJARVIS.moveHolonomic(0, power, 0);
                                    } else {
                                        robotJARVIS.moveHolonomic(0, 0, 0);
                                    }
                                }

                                if (strafeDone == true) {
                                    telemetry.addData(" ", " Strafe done");
                                } else {
                                    telemetry.addData(" ", " Strafing....");

                                }


                                if (objectHeightRatio <= targetHeightRatio) {
                                    telemetry.addData("The objectHeightRatio!", "is less than the targetHeightRatio.");
                                }
                            } else {
                                telemetry.addData("Not a skystone", " " );
                            }
                        }
                        telemetry.update();
                        }
                    } else {
                        robotJARVIS.moveHolonomic(0, 0, 0);
                    }

            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
