package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import android.graphics.Bitmap;

import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.BlockingQueue;

import java.util.List;

public class AML2Methods extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public static String skystonePosition = "notFound";


    private final int RED_THRESHOLD = 25;
    private final int GREEN_THRESHOLD = 25;
    private final int BLUE_THRESHOLD = 25;

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

    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;
    Servo found;
    Servo found2;
    Servo clamp;
    Servo capstone;
    DcMotor lift;

    public BNO055IMU imu;
    private Orientation angles;
    Acceleration gravity;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    public ElapsedTime runtime = new ElapsedTime();


    public void updateGyroValues() {
        angles = imu.getAngularOrientation();
    }

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;
    }

    public void FoundationClamp(double found1val, double found2val) {

        found.setPosition(found1val);
        found2.setPosition(found2val);


    }

    public void MoveInch(double speed, double inches) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();


        //if the position is less than the number of inches, than it sets the motors to speed
        while (Math.abs(leftBack.getCurrentPosition()) <= ticks && opModeIsActive()) {
            if (inches > 0) {
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
                    break;
                }
            }

        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

    }

    public void MoveInchGlide(double speed, double inches) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();


        //if the position is less than the number of inches, than it sets the motors to speed
        while (Math.abs(leftBack.getCurrentPosition()) <= ticks && opModeIsActive()) {
            if (inches > 0 && leftBack.getCurrentPosition() < ticks - 500) {
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
            } else if (leftBack.getCurrentPosition() >= ticks - 500) {
                leftBack.setPower(-speed * .25);
                rightBack.setPower(speed * .25);
                leftFront.setPower(-speed * .25);
                rightFront.setPower(speed * .25);

            }
            if (Math.abs(leftBack.getCurrentPosition()) >= ticks + 150) {
                break;
            }
        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void Strafe(double speed, double inches) { // to go left, set speed to a negative
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        while (Math.abs(leftBack.getCurrentPosition()) <= ticks && opModeIsActive()) {
            if (inches > 0) {
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
                    break;
                }
            }

        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void gyrostrafe(double speed, double inches) { // to go left, set speed to a negative
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        while (Math.abs(leftBack.getCurrentPosition()) <= ticks && opModeIsActive()) {
            if (inches > 0) {
                leftBack.setPower(blSpeedGyroStabilizer(.75, 0));
                rightBack.setPower(brSpeedGyroStabilizer(.75, 0, .75));
                leftFront.setPower((flSpeedGyroStabilizer(.75, 0, .75)));
                rightFront.setPower((frSpeedGyroStabilizer(.75, 0, .75)));
                if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
                    break;
                }
            }

        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void StrafeGyro(double speed, double inches, double currentAng) { // to go left, set speed to a negative
   /*     // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();


        //if the position is less than the number of inches, than it sets the motors to speed
        while (Math.abs(leftBack.getCurrentPosition()) <= ticks && opModeIsActive()) {
            if (inches > 0) {
                if (getGyroYaw() < currentAng) {
                    leftFront.setPower(-speed);
                    telemetry.addData("leftFrontPow:", -speed);
                    if (Math.abs(speed - ((currentAng - getGyroYaw()) * .01)) < .2) {
                        if (speed - ((currentAng - getGyroYaw()) * .01) > 0) {
                            leftBack.setPower(.2);
                            telemetry.addData("leftBackPow:", 0.2);
                        } else {
                            leftBack.setPower(-.2);
                            telemetry.addData("leftBackPow:", -0.2);

                        }
                    } else {
                        leftBack.setPower(speed - ((currentAng - getGyroYaw()) * .01));
                        telemetry.addData("leftBackPow:", speed - ((currentAng - getGyroYaw()) * .01));

                    }
                    rightBack.setPower(speed);
                    telemetry.addData("rightBackPow:", speed);

                    if (Math.abs(-speed - ((currentAng - getGyroYaw()) * .01)) < .2) {
                        if (-speed - ((currentAng - getGyroYaw()) * .01) > 0) {
                            rightFront.setPower(.2);
                            telemetry.addData("rightFrontPow:", 0.2);

                        } else {
                            rightFront.setPower(-.2);
                            telemetry.addData("rightFrontPow:", -0.2);

                        }
                    } else {
                        rightFront.setPower(-speed - ((currentAng - getGyroYaw()) * .01));
                        telemetry.addData("rightFrontPow:", -speed - ((currentAng - getGyroYaw()) * .01));

                    }

                } else if (getGyroYaw() > currentAng) {
                    leftFront.setPower(-speed + ((getGyroYaw() - currentAng) * .01));
                    leftBack.setPower(speed);
                    rightBack.setPower(speed + ((getGyroYaw() - currentAng) * .01));
                    rightFront.setPower(-speed);
                    telemetry.addData("leftFrontPow:", -speed + ((getGyroYaw() - currentAng) * .01));
                    telemetry.addData("leftBackPow:", speed);
                    telemetry.addData("rightBackPow:", speed + ((getGyroYaw() - currentAng) * .01));
                    telemetry.addData("rightFrontPow:", -speed);
                } else {
                    leftFront.setPower(-speed);
                    leftBack.setPower(speed);
                    rightBack.setPower(speed);
                    rightFront.setPower(-speed);
                    telemetry.addData("leftFrontPow:", -speed);
                    telemetry.addData("leftBackPow:", speed);
                    telemetry.addData("rightBackPow:", speed);
                    telemetry.addData("rightFrontPow:", -speed);

                }
            }
            if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
                break;

            }

            telemetry.addData("imuYawAng:", getGyroYaw());
            telemetry.update();
        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }*/


    }

    public double blSpeedGyroStabilizer(double speed, double currentAng) {
        double blSpeed;
        if (getGyroYaw() < currentAng) {
            blSpeed = speed + .05;
        } else if (getGyroYaw() > currentAng) {
            blSpeed = speed;
        } else {
            blSpeed = speed;
        }
        return blSpeed;

    }

    public double brSpeedGyroStabilizer(double speed, double currentAng, double brSpeed) {
        if (getGyroYaw() < currentAng) {
            brSpeed = -speed - .05;
        } else if (getGyroYaw() > currentAng) {
            brSpeed = -speed;
        } else {
            brSpeed = -speed;
        }
        return brSpeed;
    }

    public double flSpeedGyroStabilizer(double speed, double currentAng, double fLSpeed) {
        if (getGyroYaw() < currentAng) {
            fLSpeed = -speed;
        } else if (getGyroYaw() > currentAng) {
            fLSpeed = -speed - .05;
        } else {
            fLSpeed = -speed;
        }
        return fLSpeed;
    }

    public double frSpeedGyroStabilizer(double speed, double currentAng, double frSpeed) {
        if (getGyroYaw() < currentAng) {
            frSpeed = speed;
        } else if (getGyroYaw() > currentAng) {
            frSpeed = speed + .05;
        } else {
            frSpeed = speed;
        }
        return frSpeed;
    }

    public void GyroStablilizer2(double speed) {
        if (getGyroYaw() > 0) {
            leftBack.setPower(speed);
        }
    }


    public void ready() {
        while (!isStopRequested()) {
            found = hardwareMap.servo.get("found1");
            found2 = hardwareMap.servo.get("found2");
            clamp = hardwareMap.servo.get("clamp");
            leftBack = hardwareMap.dcMotor.get("leftback");
            leftFront = hardwareMap.dcMotor.get("leftfront");
            rightBack = hardwareMap.dcMotor.get("rightback");
            rightFront = hardwareMap.dcMotor.get("rightfront");
            lift = hardwareMap.dcMotor.get("lift");
            capstone = hardwareMap.servo.get("capstone");

            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            found.setPosition(1);
            found2.setPosition(0);
            clamp.setPosition(0);
            capstone.setPosition(0.3);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = this.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }
            initVision();
            telemetry.addData("imuYawAng:", getGyroYaw());
            telemetry.update();
            break;
        }
    }

    public void liftPower(double encoderChange, double power) {
        if (opModeIsActive() && (lift.getCurrentPosition() < encoderChange || lift.getCurrentPosition() > encoderChange)) {
            if (lift.getCurrentPosition() < encoderChange) {
                while (opModeIsActive() && lift.getCurrentPosition() <= encoderChange - 15) {
                    lift.setPower(power);
                }
            } else if (lift.getCurrentPosition() > encoderChange) {
                while (opModeIsActive() && lift.getCurrentPosition() >= encoderChange + 15) {
                    lift.setPower(-power);
                }
            }
        }
        lift.setPower(0);

        /*lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        if (encoderChange > 0) {
            while (lift.getCurrentPosition() < encoderChange && opModeIsActive())
                lift.setPower(power);
        } else {
            while (lift.getCurrentPosition() > encoderChange && opModeIsActive())
                lift.setPower(-power);
        }
        lift.setPower(0);*/

    }

    public void GrabBrick(double targetpos) {
        runtime.reset();
        while (runtime.milliseconds() < 1000 && opModeIsActive()) {
            clamp.setPosition(targetpos);
        }
    }


    public void PID(int heading) {
        double error = heading - getGyroYaw();

    }

    public void initVision() {
// The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
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
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public String SkystoneVision() {

        String pos = "not found yet";

        if (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        if (recognition.getLeft() < 0.2 * recognition.getImageHeight()) {
                            pos = "left";
                        } else if (recognition.getLeft() > 0.2 * recognition.getImageWidth() && recognition.getLeft() < 0.5 * recognition.getImageWidth()) {
                            pos = "middle";
                        } else {
                            pos = "right";
                        }
                        telemetry.addData("SkystonePos:", pos);
                        telemetry.update();
                    }
                }

            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }
        return pos;

    }

    public void initializeVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        params.vuforiaLicenseKey = "Acwi41P/////AAABmXAF5Uahj0aglVwEx0GLTotkFwuYvGa385NRnC3GmFdHiha7BKdStHJwB6nj4zrSBLOJ0jGEICqTReR3LiErc63MaNJf8NR/J8TUk6MOaF8xM5fa5uDU3J/7/tys+Hu1G5nlncWy3gGsHrU8lwG/rL+G0R/caVfNp0GfRtpcH7LMLDZOslSc+URv9+IF8+C0jA4JzTfM4lRkOEcIqIyTs20EZC+W3QYI7o7n700hOwq+WpoG7qMgqcrgk3+B1/hTLICE3fodM/34CQjbEONYKpGbj8IOG714CeY9qyI6WhainXidKda/QAslXEvYCDvBCZoGW/4I3TaZAJUWAeD1l5SeL/m4nuJxV9Jmai/0/9Qn";;
        params.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(params);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);
        vuforia.enableConvertFrameToBitmap();

    }

    public Bitmap getBitmap() throws InterruptedException {

        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        telemetry.addData("Num images", numImages);
        telemetry.update();

        for (int i = 0; i < numImages; i++) {

            int format = picture.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;
            } else {
                telemetry.addLine("Didn't find correct RGB format");
                telemetry.update();


            }
        }

        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());


        sleep(500);

        picture.close();


        return imageBitmap;
    }

    public double getImageHeight() throws InterruptedException {
        Bitmap bitmap = getBitmap();
        return bitmap.getHeight();
    }

    public double getImageWidth() throws InterruptedException {
        Bitmap bitmap = getBitmap();
        return bitmap.getWidth();
    }

    //True for Blue
    public String Skystone(boolean red) throws InterruptedException {
        if (opModeIsActive()) {
            double avgX = 0;
            double avgY = 0;
            double medX = 0;
            double medY = 0;
            Bitmap bitmap = getBitmap();
            int skystonePixelCount = 0;
            ArrayList<Integer> xValues = new ArrayList<>();
            ArrayList<Integer> yValues = new ArrayList<>();

            for (int y = 0; y < bitmap.getHeight(); y++) {
                for (int x = 0; x < bitmap.getWidth(); x++) {
                    int pixel = bitmap.getPixel(x, y);
                    if (red(pixel) <= RED_THRESHOLD && blue(pixel) <= BLUE_THRESHOLD && green(pixel) <= GREEN_THRESHOLD) {
                        xValues.add(x);
                        yValues.add(y);
                    }
                }
            }

            for (int xCoor : xValues) {
                avgX += xCoor;
            }
            for (int yCoor : yValues) {
                avgY += yCoor;
            }
            Collections.sort(xValues);
            Collections.sort(yValues);
            medX = xValues.get(xValues.size() / 2);
            medY = yValues.get(yValues.size() / 2);
            avgX /= xValues.size();
            avgY /= yValues.size();
            if (red) {
                if (medX < bitmap.getWidth() * 0.33333) {
                    skystonePosition = "1 & 4";
                    telemetry.addData("skystonePosition: ", skystonePosition);
                } else if (medX < bitmap.getWidth() * 0.666666 && medX > bitmap.getWidth() * 0.3333333) {
                    skystonePosition = "2 & 5";
                    telemetry.addData("skystonePosition: ", skystonePosition);
                } else {
                    skystonePosition = "3 & 6";
                    telemetry.addData("skystonePosition: ", skystonePosition);
                }
                telemetry.update();
            } else {
                if (medX < bitmap.getWidth() * 0.33333) {
                    skystonePosition = "3 & 6";
                    telemetry.addData("skystonePosition: ", skystonePosition);
                } else if (medX < bitmap.getWidth() * 0.666666 && medX > bitmap.getWidth() * 0.3333333) {
                    skystonePosition = "2 & 5";
                    telemetry.addData("skystonePosition: ", skystonePosition);
                } else {
                    skystonePosition = "1 & 4";
                    telemetry.addData("skystonePosition: ", skystonePosition);
                }
                telemetry.update();
            }
        }
        return skystonePosition;
    }



    public void runOpMode() throws InterruptedException {

    }


}
