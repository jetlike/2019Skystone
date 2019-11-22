package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Methods.AML2Methods;

import java.util.List;

@Autonomous(name = "testany", group = "test")
public class TestAny extends AML2Methods {
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    double left = 0;
    double right = 0;
    double top = 0;
    double bottom = 0;
    double confidence = 0;

    private AML2Methods aml2Methods = new AML2Methods();
    private WhatToDo whattodo = new WhatToDo();
    private AML2QuarryAutoRed aml2QuarryAutoRed = new AML2QuarryAutoRed();
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
            "ARp3PIv/////AAABmcDAZ43OoEcolhbbJCBtrHUKIouLxMQfnSzi36d3LvbildUpTTAwmL/WabNoiOwEMIuNDpSQE2647Jt+m2j8LKsNjVJ5WKt+HvYD3w5PZxXlWF8ZQodb5PCq+hE/1daSB8Aup6IyH3/9nn7JWrrDRZDHp+Pz40zlsVNIUNZObH6nEbS8mP+H+BxsnvUcymn1wEVuOwVW8fH7nmFCR5ps1uD3ZeVi+yCL8NaILGFVZik/isalrDaS/b9hwX8rc4AfMMl6FMjnc4inuNlIa8eo/cHVX5v3uZNhEjRSw8M9ZW7O3eoIKXX+izDovwJFbF6dhaVQxRw1onhu02wJRQkD8YiQgqYZMxqqqVGmI0K6rMQ/";

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
    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    public void runOpMode() throws InterruptedException {
        ready();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                right = recognition.getRight();
                                left = recognition.getLeft();
                                top = recognition.getTop();
                                bottom = recognition.getBottom();
                                confidence = recognition.getConfidence();
                            }
                            telemetry.update();


                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }



            break;
        }
    }

}



