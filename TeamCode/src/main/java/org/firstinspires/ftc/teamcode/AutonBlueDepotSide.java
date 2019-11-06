/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.NotOpMode.FritsBot;

import java.util.List;


@Autonomous(name="RedDepotSide", group="red")
public class AutonRedDepotSide extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AYrJcPz/////AAABmeoL9kpl/ELOsQ43TkTjFAxba3UVdN2Xo71qDtfCBkTKYPkUZOWTbJ3AgYW0HtpPZ1pecxwUjMYiN4BtLt32s097m/E+/LUSLc6waPrJe/fnSekZxd7WUkU8Fb/f6CZLthxCrKt5nzdCx2LLg3Sfjpegd29NVZDhG/oZYD6wYp28jFqqsHzJ6D1v8RcoRYmdXPpNHFjU0dW4pj2i+wwdVZtWRDixBIb+79fnNT3EQf9E897pOf5Ea30Td1uDwotmvt78uKUe7Hi1Z/7pfBtLCgemUJSlLuA5Fviuesy3kTifpwLH3m1pIcTo8tzwwc7NPa8YG9PFsivWf6+wyW2NOlT9JwaBWvgbs6oy8kdsoc+x";
    String skystonePos;

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

    private FritsBot robot = new FritsBot();

    @Override
    public void runOpMode() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        if (tfod != null) {
            tfod.activate();
        }

        robot.init(hardwareMap);
        robot.liftGrab.moveGrabber("in", "open");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.sideGrabber.moveGrabber("up", "open");
        robot.drivetrain.encoderDrive(0.5, 0.54, 0.75, 64, 65, -71, 5, this);

        if(opModeIsActive()) {
            while (skystonePos == null && opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        if (updatedRecognitions.size() == 2) {
                            Recognition recognitionOne = updatedRecognitions.get(0);
                            Recognition recognitionTwo = updatedRecognitions.get(1);
                            if (recognitionOne.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                if (recognitionOne.getLeft() < recognitionTwo.getLeft()) {
                                    skystonePos = "left";
                                } else {
                                    skystonePos = "mid";
                                }
                            } else if (recognitionTwo.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                if (recognitionTwo.getLeft() < recognitionOne.getLeft()) {
                                    skystonePos = "left";
                                } else {
                                    skystonePos = "mid";
                                }
                            } else {
                                skystonePos = "right";
                            }
                        }

                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        telemetry.addData("position", skystonePos);
        telemetry.update();

        if(skystonePos.equals("left")) {
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0, -4, -4, 0, 1, this);
            robot.drivetrain.encoderDrive(0, 0, 0.4, 0, 0, -25, 4, this);
            robot.sideGrabber.moveGrabber("down", "open");
            sleep(1000);
            robot.sideGrabber.moveGrabber("down", "closed");
            sleep(300);
            robot.sideGrabber.moveGrabber("up", "closed");
            robot.drivetrain.encoderDrive(0, 0, 0.7, 0, 0, 20, 3, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0, -15, -15, 0, 4, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0, 36, -36, 0, 4, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0.6, 13, 14, -250, 4, this);
            robot.drivetrain.encoderDrive(0.6, 0.6, 0.5, -225, -225, -20, 4, this);
            robot.sideGrabber.moveGrabber("down", "open");
            sleep(300);
            robot.sideGrabber.moveGrabber("up", "open");
            robot.drivetrain.encoderDrive(0.6, 0.6, 0, 50, 50, 0, 4, this);


        }
        else if(skystonePos.equals("mid")) {
            robot.drivetrain.encoderDrive(0.6, 0.6, 0, -23, -23, 0, 1, this);
            robot.drivetrain.encoderDrive(0, 0, 0.7, 0, 0, -25, 4, this);
            robot.sideGrabber.moveGrabber("down", "open");
            sleep(1000);
            robot.sideGrabber.moveGrabber("down", "closed");
            sleep(300);
            robot.sideGrabber.moveGrabber("up", "closed");
            robot.drivetrain.encoderDrive(0, 0, 0.5, 0, 0, 20, 3, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0, -15, -15, 0, 4, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0, 36, -36, 0, 4, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0.6, 13, 14, -250, 4, this);
            robot.drivetrain.encoderDrive(0.6, 0.6, 0.5, -200, -200, -20, 4, this);
            robot.sideGrabber.moveGrabber("down", "open");
            sleep(300);
            robot.sideGrabber.moveGrabber("up", "open");
            robot.drivetrain.encoderDrive(0.6, 0.6, 0, 50, 50, 0, 4, this);



        }
        else {
            robot.drivetrain.encoderDrive(0.6, 0.6, 0, -43, -43, 0, 1, this);
            robot.drivetrain.encoderDrive(0, 0, 0.7, 0, 0, -26, 4, this);
            robot.sideGrabber.moveGrabber("down", "open");
            sleep(1000);
            robot.sideGrabber.moveGrabber("down", "closed");
            sleep(300);
            robot.sideGrabber.moveGrabber("up", "closed");
            robot.drivetrain.encoderDrive(0, 0, 0.5, 0, 0, 20, 3, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0, -15, -15, 0, 4, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0, 36, -36, 0, 4, this);
            //robot.drivetrain.encoderDrive(0.6, 0.6, 0.6, 13, 14, -250, 4, this);
            robot.drivetrain.encoderDrive(0.6, 0.6, 0.5, -180, -180, -20, 4, this);
            robot.sideGrabber.moveGrabber("down", "open");
            sleep(300);
            robot.sideGrabber.moveGrabber("up", "open");
            robot.drivetrain.encoderDrive(0.6, 0.6, 0, 50, 50, 0, 4, this);

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
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
