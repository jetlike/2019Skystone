package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "godlyAlliance", name = "Park")
public class Park extends AML2Methods {


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {


        ready();

        // wait for start button.

        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        while(!isStopRequested() && opModeIsActive())
        MoveInch(.6, 12);
        telemetry.addData("moving forward to park:", "autocomplete");
        telemetry.update();

    }
}