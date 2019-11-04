package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Methods extends LinearOpMode {
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;
    Servo found;
    Servo found2;
    Servo clamp;
    Servo capstone;
    DcMotor lift;
    public double error;
    public int LiftDown = 200;
    public int LiftUp = 1000;
    private Orientation angles;

    public BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.30, correction;

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
            }
            if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
                break;
            }
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
        }
    }

    public void StrafeGyro(double speed, double inches, double currentAng) { // to go left, set speed to a negative
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
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
                    if (Math.abs(speed - ((currentAng - getGyroYaw()) * .01)) < .2) {
                        if (speed - ((currentAng - getGyroYaw()) * .01) > 0)
                            leftBack.setPower(.2);
                        else
                            leftBack.setPower(-.2);
                    } else {
                        leftBack.setPower(speed - ((currentAng -getGyroYaw()) * .01));
                    }
                    rightBack.setPower(speed);
                    if (Math.abs(-speed - ((currentAng -getGyroYaw()) * .01)) <.2){
                        if (-speed - ((currentAng -getGyroYaw()) * .01) >0)
                        rightFront.setPower(.2);
                        else
                        rightFront.setPower(-.2);
                    } else{
                        rightFront.setPower(-speed - ((currentAng -getGyroYaw()) * .01));
                    }

                } else if (getGyroYaw() > currentAng){
                    leftFront.setPower(-speed + ((getGyroYaw() - currentAng) * .01));
                    leftBack.setPower(speed);
                    rightBack.setPower(speed + ((getGyroYaw() - currentAng) * .01));
                    rightFront.setPower(-speed);
                } else{
                    leftFront.setPower(-speed);
                    leftBack.setPower(speed);
                    rightBack.setPower(speed);
                    rightFront.setPower(-speed);
                }
            }
            if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
                break;

            }


        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
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

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
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

    public void runOpMode() {
    }

    public void PID(int heading) {
    }
}
