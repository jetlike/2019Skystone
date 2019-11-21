package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Methods.AML2Methods;

@Autonomous(group = "godlyQuarry", name = "BitmapQuarryAutoBlue")
public class AML2QuarryAutoBlueBitmap extends AML2Methods {


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {


        ready();

        // wait for start button.

        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {
        }
    }
}