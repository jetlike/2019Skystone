package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "testany", group = "test")
public class TestAny extends Methods{
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        ready();

        waitForStart();


        while(!isStopRequested() && opModeIsActive())
        gyrostrafe(.75,20);}
    }

