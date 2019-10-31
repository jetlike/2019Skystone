package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MecTeleOp", group = "godly")
public class MecTeleOp extends OpMode {

    DcMotor lift;           //declaring stuff
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;

    Servo clamp;
    double lasta = 0;
    boolean clampb = true;

    Servo foundation1;
    Servo foundation2;
    double foundb = 0;
    boolean foundup = true;

    private ElapsedTime runtime = new ElapsedTime();


    public void init() {
        lift = hardwareMap.dcMotor.get("lift");        //initialize motors, servos
        lf = hardwareMap.dcMotor.get("leftfront");
        rf = hardwareMap.dcMotor.get("rightfront");
        lb = hardwareMap.dcMotor.get("leftback");
        rb = hardwareMap.dcMotor.get("rightback");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        clamp = hardwareMap.servo.get("clamp");
        foundation1 = hardwareMap.servo.get("found1");
        foundation2 = hardwareMap.servo.get("found2");

        clamp.setPosition(0.1); //this is when clampb is true

        foundation1.setPosition(1);  //when foundup is true
        foundation2.setPosition(0);
    }

    public void loop() {

        if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
            double FLP = gamepad1.left_stick_y + -gamepad1.left_stick_x - gamepad1.right_stick_x;
            double FRP = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            double BLP = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            double BRP = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;  //checks if sticks have been moved
            //then calculates power based on movement of sticks
            double max = Math.max(Math.max(Math.abs(FLP), Math.abs(FRP)), Math.max(Math.abs(BLP), Math.abs(BRP)));
            //calculates the max of the powers, used to see if
            if (max > 1) {                                                        //scaling of power is needed
                FLP /= max;    //if scaling is needed this does the job
                FRP /= max;
                BLP /= max;
                BRP /= max;
            }



            if (gamepad1.right_trigger > 0.1) { //if you want to go slower hold down right trigger
                lf.setPower(FLP*.35); //ez stuff
                rf.setPower(FRP*.35);
                lb.setPower(BLP*.35);
                rb.setPower(BRP*.35);
                telemetry.addData("FrontLeftPow:", FLP*.35); //add telemetry to see how much power each motor is getting
                telemetry.addData("FrontRightPow:", FRP*.35);
                telemetry.addData("BackLeftPow:", BLP*.35);
                telemetry.addData("BackRightPow:", BRP*.35);
            } else {
                lf.setPower(FLP); //ez stuff
                rf.setPower(FRP);
                lb.setPower(BLP);
                rb.setPower(BRP);
                telemetry.addData("FrontLeftPow:", FLP); //add telemetry to see how much power each motor is getting
                telemetry.addData("FrontRightPow:", FRP);
                telemetry.addData("BackLeftPow:", BLP);
                telemetry.addData("BackRightPow:", BRP);
            }

        } else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }


        if (Math.abs(gamepad2.right_stick_y) > .1) {            //lift code;easy stuff
            lift.setPower(gamepad2.right_stick_y);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y); //add telemetry to see how much power lift is getting
        } else {
            lift.setPower(0);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y); //add telemetry to see how much power lift is getting

        }

        if (gamepad2.a) {                                              //clamp code, checks if the a button has been pressed
            if (clampb && runtime.milliseconds() > lasta + 500) {      //once pressed, will check whether clampb is true or false
                clamp.setPosition(0.85);                       //makes movements based on the clampb boolean
                clampb = false;                              // checks if the last time you've hit the button has been more than
                runtime.reset();                            // x amount of seconds, so it doesn't jitter
                telemetry.addData("ClampPos:", 0.6); //add telemetry to see where clamp is positioned
            } else if (!clampb && runtime.milliseconds() > lasta + 500) {
                clamp.setPosition(0.1);
                clampb = true;
                runtime.reset();
                telemetry.addData("ClampPos:", 0.1); //add telemetry to see where clamp is positioned
            }
        }

        if (gamepad2.b) {                                         //foundation servo code, checks if the b button has been pressed
            if (foundup && runtime.milliseconds() > foundb + 500) {
                foundation1.setPosition(.55);             //once pressed, will check whether or not foundup is true or false
                foundation2.setPosition(0.45);              //makes movements based on the foundup boolean
                foundup = false;                        // checks if the last time you've hit the button has been more than
                runtime.reset();                        // x amount of seconds, so it doesn't jitter
                telemetry.addData("FoundationPos:", 0.8); //add telemetry to ses where foundation servos are positioned
            } else if (!foundup && runtime.milliseconds() > foundb + 500) {
                foundation1.setPosition(1);
                foundation2.setPosition(0);
                foundup = true;
                runtime.reset();
                telemetry.addData("FoundationPos:", 0.1); //add telemetry to ses where foundation servos are positioned
            }
        }

        telemetry.update(); //have to display the added data

    }

    public void stop() {

    }
}
