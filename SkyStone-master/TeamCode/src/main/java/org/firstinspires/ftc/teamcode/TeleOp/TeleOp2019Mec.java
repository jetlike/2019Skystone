package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "MecTeleOp", group = "2019")
public class TeleOp2019Mec extends OpMode {
    DcMotor lb;
    DcMotor rb;
    DcMotor lf;
    DcMotor rf;
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor lift;
    Servo clamp;
    boolean clampb = false;


    public void init() {
        lb = hardwareMap.dcMotor.get("Left Back");
        rb = hardwareMap.dcMotor.get("Right Back");
        lf = hardwareMap.dcMotor.get("Left Front");
        rf = hardwareMap.dcMotor.get("Right Front");
        lift = hardwareMap.dcMotor.get("left Lift");
        clamp = hardwareMap.servo.get("clamp");
        lift.setPower(0);
    }

    public void loop() {


        if (Math.abs(gamepad1.left_stick_y) > .05) {
            lb.setPower(gamepad1.left_stick_y);
            rb.setPower(-gamepad1.left_stick_y);
            lf.setPower(gamepad1.left_stick_y);
            rf.setPower(-gamepad1.left_stick_y);
        } else {
            lb.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
        }
        if (Math.abs(gamepad1.left_stick_x) > .05) {
            lf.setPower(-gamepad1.left_stick_x);
            lb.setPower(gamepad1.left_stick_x);
            rf.setPower(-gamepad1.left_stick_x);
            rb.setPower(gamepad1.left_stick_x);
        } else {
            lb.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
        }
        if (Math.abs(gamepad1.right_stick_x) > .05) {
            lb.setPower(-gamepad1.right_stick_x);
            rb.setPower(-gamepad1.right_stick_x);
            lf.setPower(-gamepad1.right_stick_x);
            rf.setPower(-gamepad1.right_stick_x);
        } else {
            lb.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
        }
        if ((gamepad2.right_stick_y) > .1) {
            lift.setPower(1);

        //    telemetry.addData("Lift Value:", gamepad2.right_stick_y);
            telemetry.update();
        } else if ((gamepad2.right_stick_y) < -0.1){
            lift.setPower(-1);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y);
            telemetry.update();
        } else {
            lift.setPower(0);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y);
            telemetry.update();
        }


        if (gamepad2.a) {
            if (!clampb) {
                // while (gamepad2.a);
                clamp.setPosition(.8);
                clampb = true;
            } else if (clampb) {
                //  while (gamepad2.a);
                clamp.setPosition(0.1);
                clampb = false;
            }

        }

    }

    public void stop() {

    }
}
