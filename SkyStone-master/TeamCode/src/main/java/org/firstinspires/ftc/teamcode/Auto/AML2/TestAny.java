package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Methods.AML2Methods;

@Autonomous(name = "testany", group = "test")
public class TestAny extends AML2Methods {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        ready();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
            turnPD(90, .4,.45,2); // .4p and .35d?
            break;
        }

    }
}


