package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "testany", group = "test")
public class TestAny extends AML2Methods{
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        ready();

        waitForStart();


        while(!isStopRequested() && opModeIsActive()){

            Skystone(false);
        }

    }
}


