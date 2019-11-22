package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionWebcam;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Methods.AML2Methods;

import java.util.List;

@Autonomous(group = "godlyQuarry", name = "QuarryAutoRed")
public class AML2QuarryAutoRed extends AML2Methods {

    private ElapsedTime runtime = new ElapsedTime();
    private WhatToDo whattodo = new WhatToDo();



    public void runOpMode() {

        ready();


        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {

            SkystoneVision();
            liftPower(299, 1); //have to do it here, because for some whack reason, it doesn't work in the ready method

        //    whattodo.ProcessVision();
            if (whattodo.results.strafe == .5) {
                Strafe(.5, .5);
            }
            if (whattodo.results.strafe == .5) {
                Strafe(-.5, .5);
            }
            if (whattodo.results.forward == .2) {
                MoveInch(.2, .5);
            }
            if (whattodo.results.forward == .5) {
                MoveInch(.5, .5);
            }
            if (whattodo.results.grab == true) {
                GrabBrick(.7);
            }
        }

     /*       MoveInch(.4, 27.5);
            sleep(400);

            GrabBrick(.7);
            sleep(400);

            liftPower(10, 1);
            sleep(400);

            MoveInch(-1, 9.3);
            sleep(400);

            Strafe(.5, 1);
            sleep(200);

            GrabBrick(.45);
            sleep(200);

            MoveInch(-1, 4);
            sleep(400);

            Strafe(-0.5, 57.5);
            sleep(400);

            liftPower(299, 0.4);
            sleep(400);

            MoveInchGlide(0.43, 14.3);
            sleep(400);

            GrabBrick(.7);
            sleep(400);

            liftPower(10, .4);
            sleep(400);

            MoveInch(-.8, 8);
            sleep(400);

            Strafe(.5, 48.8);
            break;
            */

    }
}


