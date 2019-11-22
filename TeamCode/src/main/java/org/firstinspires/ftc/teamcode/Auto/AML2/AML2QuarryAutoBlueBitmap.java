package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Methods.AML2Methods;

@Autonomous(name = "QuarryBlueBitmap", group = "godlyBitmap")
public class AML2QuarryAutoBlueBitmap extends AML2Methods {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        ready();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
            MoveInch(.7, 12);
            sleep(200); //not needed later

            //  GrabBrick(0.45);
            //  sleep(200);

            switch (Skystone(false)) {
                case "3 & 6":
//----------------------------------------LineUpWithBlock-------------------------------------------
                    Strafe(-0.3, 2);
                    sleep(200);

                    MoveInch(.5, 15);
                    sleep(200);
//---------------------------------------GrabBlock--------------------------------------------------
                    //  GrabBrick(.7);
                    //  sleep(200);
//---------------------------------------DeliverBlock-----------------------------------------------
                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(-90, .4, .45, 3);
                    sleep(200);

                    MoveInch(.8, 30);
                    sleep(200);

                    //   GrabBrick(0.45);
                    //   sleep(200);
//---------------------------------------Get2ndBlock(Deliver and Park)-------------------------------
                    MoveInch(-0.8, 52);
                    sleep(200);

                    turnPD(0, .4, .45, 3);
                    sleep(200);

                    MoveInch(0.5, 6);
                    sleep(200);

                    GrabBrick(0.7);
                    sleep(200);

                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(-90, .4, .45, 3);
                    sleep(200);

                    MoveInch(1, 46);

                case "2 & 5":
//----------------------------------------LineUpWithBlock-------------------------------------------
                    Strafe(0.3, 6);
                    sleep(200);

                    MoveInch(.5, 15);
                    sleep(200);
//---------------------------------------GrabBlock--------------------------------------------------
                    //  GrabBrick(.7);
                    //  sleep(200);
//---------------------------------------DeliverBlock-----------------------------------------------
                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(-90, .4, .45, 3);
                    sleep(200);

                    MoveInch(.8, 38);
                    sleep(200);

                    //   GrabBrick(0.45);
                    //   sleep(200);
//--------------------------------------Get2ndBlock(Deliver and Park)-------------------------------
                    MoveInch(-0.8, 60);
                    sleep(200);

                    turnPD(0, .4, .45, 3);
                    sleep(200);

                    MoveInch(0.5, 6);
                    sleep(200);

                    GrabBrick(0.7);
                    sleep(200);

                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(-90, .4, .45, 3);
                    sleep(200);

                    MoveInch(1, 54);

                case "1 & 4":
//----------------------------------------LineUpWithBlock-------------------------------------------
                    Strafe(0.3, 14);
                    sleep(200);

                    MoveInch(.5, 15);
                    sleep(200);
//---------------------------------------GrabBlock--------------------------------------------------
                    //  GrabBrick(.7);
                    //  sleep(200);
//---------------------------------------DeliverBlock-----------------------------------------------
                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(-90, .4, .45, 3);
                    sleep(200);

                    MoveInch(.8, 46);
                    sleep(200);

                    //   GrabBrick(0.45);
                    //   sleep(200);
//---------------------------------------Get2ndBlock(Deliver and Park)------------------------------
                    MoveInch(-0.8, 68);
                    sleep(200);

                    turnPD(0, .4, .45, 3);
                    sleep(200);

                    MoveInch(0.5, 6);
                    sleep(200);

                    GrabBrick(0.7);
                    sleep(200);

                    MoveInch(-0.7, 6);
                    sleep(200);

                    turnPD(-90, .4, .45, 3);
                    sleep(200);

                    MoveInch(1, 62);
            }

            break;
        }

    }
}


