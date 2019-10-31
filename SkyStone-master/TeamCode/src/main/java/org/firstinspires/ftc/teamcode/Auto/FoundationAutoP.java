package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "godly", name = "FoundationAutoP")
public class FoundationAutoP extends LinearOpMode {
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;
    Servo found;
    Servo found2;
    Servo clamp;
    DcMotor lift;
    private ElapsedTime runtime = new ElapsedTime();

    public void MoveInch(double speed, double inches) {
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
        while (Math.abs(leftBack.getCurrentPosition()) <= ticks) {
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


    public void runOpMode() {
        found = hardwareMap.servo.get("found1");
        found2 = hardwareMap.servo.get("found2");
        clamp = hardwareMap.servo.get("clamp");
        leftBack = hardwareMap.dcMotor.get("leftback");
        leftFront = hardwareMap.dcMotor.get("leftfront");
        rightBack = hardwareMap.dcMotor.get("rightback");
        rightFront = hardwareMap.dcMotor.get("rightfront");
        lift = hardwareMap.dcMotor.get("lift");


        found.setPosition(1);
        found2.setPosition(0);
        clamp.setPosition(0.1);



        waitForStart();

        MoveInch(1, 12.5);
        telemetry.addData("Running MoveInch:", "1");
        telemetry.update();
        sleep(1000);

        Clamp(0.5, 0.5);
        telemetry.addData("Running Clamp:", "1");
        telemetry.update();
        sleep(1000);

        MoveInch(-.5, 26);
        telemetry.addData("Running MoveInch:", "1");
        telemetry.update();
        sleep(1000);

        Clamp(1, 0);
        telemetry.addData("Running Clamp:", "1");
        telemetry.update();
        sleep(1000);

        strafe(.5, 22);
        telemetry.addData("Running strafe:", "1");
        telemetry.update();
        sleep(1000);

        MoveInch(1, 7);
        telemetry.addData("Running MoveInch:", "1");
        telemetry.update();
        sleep(1000);

        strafe(-1, 6);
        telemetry.addData("Running strafe:", "1");
        telemetry.update();
        sleep(1000);

        MoveInch(-1, 7);
        telemetry.addData("Running MoveInch:", "1");
        telemetry.update();
        sleep(1000);

        strafe(1, 15);
        telemetry.addData("Running strafe:", "1");
        telemetry.update();
        sleep(1000);


    }

    public void Clamp(double targetPos1, double targetPos2) {

        found.setPosition(targetPos1);
        found2.setPosition(targetPos2);


    }

    public void strafe(double speed, double inches) { // to go left, set speed to a negative
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
        while (Math.abs(leftBack.getCurrentPosition()) <= ticks) {
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
}


