package org.firstinspires.ftc.teamcode.Test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Lift", group = "testing")
public class Lift extends OpMode {

    DcMotor biglift;
    private ElapsedTime runtime = new ElapsedTime();

    public void init() {
        biglift = hardwareMap.dcMotor.get("lift");
    }

    public void loop() {
        if (Math.abs(gamepad2.right_stick_y) > .1) {
            biglift.setPower(gamepad2.right_stick_y);
        }
    }


}
