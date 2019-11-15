package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "godlyQuarry", name = "QuarryAutoBlue")
public class AML2QuarryAutoBlue extends AML2Methods {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        ready();


        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {


            liftPower(299, 1); //have to do it here, because for some whack reason, it doesn't work in the ready method


            MoveInch(.4, 27);
            sleep(400);

            GrabBrick(.7);
            sleep(400);

            liftPower(10, 1);
            sleep(400);

            MoveInch(-1, 8);
            sleep(400);

            Strafe(-.5, 42);
            sleep(200);

            GrabBrick(0.45);
            sleep(200);

            MoveInch(-1, 4);
            sleep(400);

            Strafe(0.5, 57.7);
            sleep(400);

            liftPower(299, 0.4);
            sleep(400);

            MoveInchGlide(0.43, 10.6);
            sleep(400);

            GrabBrick(.7);
            sleep(400);

            liftPower(10, .4);
            sleep(400);

            MoveInch(-.8, 8);
            sleep(400);

            Strafe(-.5, 54.5);
            break;
        }
    }


}


