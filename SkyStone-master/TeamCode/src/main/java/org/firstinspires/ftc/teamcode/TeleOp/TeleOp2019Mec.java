package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleOp2019Mec", group = "2019")
public class TeleOp2019Mec extends OpMode {
    DcMotor lb;
    DcMotor rb;
    DcMotor lf;
    DcMotor rf;
    double lasta = 0;
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor lift;
    Servo clamp;
    boolean clampb = false;


    public void init() {
        lb = hardwareMap.dcMotor.get("Left Back");
        rb = hardwareMap.dcMotor.get("Right Back");
        lf = hardwareMap.dcMotor.get("Left Front");
        rf = hardwareMap.dcMotor.get("Right Front");
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = hardwareMap.dcMotor.get("lift");
        clamp = hardwareMap.servo.get("clamp");

        clamp.setPosition(0.1);
    }
    @Override
    public void start() {
        lift.setPower(0);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    public void loop() {


        if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1 || (Math.abs(gamepad1.right_stick_x)) > .1) {
            double FLP = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double FRP = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            double BLP = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x; // using gears; direction reversed
            double BRP = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x; // using gears direction reversed

            double max = Math.max(Math.max(Math.abs(FLP), Math.abs(FRP)), Math.max(Math.abs(BLP), Math.abs(BRP)));
            // scales power if any motor power is greater than 1
            if (max > 1) {
                FLP /= max;
                FRP /= max;
                BLP /= max;
                BRP /= max;
            }
            lf.setPower(FLP);
            rf.setPower(FRP);
            lb.setPower(BLP);
            rb.setPower(BRP);
        }
        else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }
        if ((gamepad2.right_stick_y) > .1) {
            lift.setPower(.1);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y);
        } else if ((gamepad2.right_stick_y) < -0.1) {
            telemetry.addData("Lift Value:", gamepad2.right_stick_y);
        } else {
            lift.setPower(0);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y);
        }


        if (gamepad2.a) {
            if (!clampb && runtime.milliseconds() > lasta + 500) {
                clamp.setPosition(.8);
                clampb = true;
                runtime.reset();
            } else if (clampb && runtime.milliseconds() > lasta + 500) {
                clamp.setPosition(0.1);
                clampb = false;
                runtime.reset();
            }
        }
        telemetry.update();

    }

    public void stop() {

    }
}
