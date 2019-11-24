package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Methods.AML2Methods;

@Autonomous(name = "QuarryRedBitmap", group = "godlyBitmap")
public class AML2QuarryAutoRedBitmap extends AML2Methods {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        ready();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
            MoveInch(.7, 12);

            GrabBrick(0.45);
            sleep(200);

            switch (Skystone(false)) {
                case "3 & 6":
//----------------------------------------LineUpWithBlock-------------------------------------------
                    MoveInch(.5, 12);
                    sleep(200);
//---------------------------------------GrabBlock--------------------------------------------------
                    GrabBrick(.7);
                    sleep(200);
//---------------------------------------DeliverBlock-----------------------------------------------
                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(90, .4, .45, 3);
                    sleep(200);

                    MoveInch(.8, 34);
                    sleep(200);

                    GrabBrick(0.45);
                    sleep(200);
//---------------------------------------Get2ndBlock(Deliver and Park)-------------------------------
                    MoveInch(-0.8, 56);
                    sleep(200);

                    turnPD(0, .4, .45, 3);
                    sleep(200);

                    MoveInch(0.5, 4);
                    sleep(200);

                    GrabBrick(0.7);
                    sleep(200);

                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(90, .4, .45, 3);
                    sleep(200);

                    MoveInch(1, 58);

                    GrabBrick(0.45);
                    sleep(200);

                    MoveInch(-.3, 12);
                    break;

                case "2 & 5":
//----------------------------------------LineUpWithBlock-------------------------------------------
                    Strafe(-0.3, 6);
                    sleep(200);

                    turnPD(0, .1, .1, 1);
                    sleep(100);

                    MoveInch(.5, 12);
                    sleep(200);
//---------------------------------------GrabBlock--------------------------------------------------
                    GrabBrick(.7);
                    sleep(200);
//---------------------------------------DeliverBlock-----------------------------------------------
                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(90, .4, .45, 3);
                    sleep(200);

                    MoveInch(.8, 42);
                    sleep(200);

                    GrabBrick(0.45);
                    sleep(200);
//--------------------------------------Get2ndBlock(Deliver and Park)-------------------------------
                    MoveInch(-0.8, 62.5);
                    sleep(200);

                    turnPD(0, .4, .45, 3);
                    sleep(200);

                    MoveInch(0.5, 6);
                    sleep(200);

                    GrabBrick(0.7);
                    sleep(200);

                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(90, .4, .45, 3);
                    sleep(200);

                    MoveInch(.8, 64);

                    GrabBrick(0.45);
                    sleep(200);

                    MoveInch(-.3, 10);
                    break;

                case "1 & 4":
//----------------------------------------LineUpWithBlock-------------------------------------------
                    Strafe(-0.3, 10.5);
                    sleep(200);

                    turnPD(0, .1, .1, 1);
                    sleep(100);

                    MoveInch(.5, 12);
                    sleep(200);
//---------------------------------------GrabBlock--------------------------------------------------
                    GrabBrick(.7);
                    sleep(200);
//---------------------------------------DeliverBlock-----------------------------------------------
                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(90, .4, .45, 3);
                    sleep(200);

                    MoveInch(.8, 50);
                    sleep(200);

                    GrabBrick(0.45);
                    sleep(200);
//---------------------------------------Get2ndBlock(Deliver and Park)------------------------------
                    MoveInch(-0.8, 57);
                    sleep(200);

                    turnPD(-45, .6, .675, 3);
                    sleep(200);

                    MoveInch(0.25, 11);
                    sleep(200);

                    GrabBrick(0.7);
                    sleep(200);

                    MoveInch(-0.7, 11);
                    sleep(200);

                    turnPD(90, .6, .675, 3);
                    sleep(200);

                    MoveInch(.8, 60);

                    GrabBrick(0.45);
                    sleep(200);

                    MoveInch(-.3, 12);
                    break;

            }

            break;
        }

    }
}


