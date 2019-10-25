package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "LiftCode", group = "test")
public class LiftCode extends OpMode {

    DcMotor lift;
    private ElapsedTime runtime = new ElapsedTime();


    public void init() {
        lift = hardwareMap.dcMotor.get("lift");
    }

    public void loop() {

        if (Math.abs(gamepad2.right_stick_y) > .1) {
            lift.setPower(gamepad2.right_stick_y);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y);
        } else {
            lift.setPower(0);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y);

        }
        telemetry.update();
    }

    public void stop() {

    }
}
