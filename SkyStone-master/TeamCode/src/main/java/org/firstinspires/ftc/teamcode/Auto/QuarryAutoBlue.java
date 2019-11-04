package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(group = "godlyQuarry", name = "QuarryAutoBlue")
public class QuarryAutoBlue extends Methods {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        ready();


        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {

            liftPower(7635, 1); //have to do it here, because for some whack reason, it doesn't work in the ready method


            MoveInch(.4, 25.8);
            sleep(400);

            GrabBrick(0.85);
            sleep(400);

            liftPower(7250, 1);
            sleep(400);

            MoveInch(-1, 6);
            sleep(400);

            Strafe(-.5, 42);
            sleep(200);

            GrabBrick(0.6);
            sleep(200);

            MoveInch(-1, 4);
            sleep(400);

            Strafe(0.5, 55.25);
            sleep(400);

            liftPower(7635, 0.4);
            sleep(400);

            MoveInchGlide(0.43, 13.5);
            sleep(400);

            GrabBrick(0.85);
            sleep(400);

            liftPower(7250, .4);
            sleep(400);

            MoveInch(-.8, 8);
            sleep(400);

            Strafe(-.5, 52);
            sleep(200);

            MoveInch(.4, 4);
            break;
        }
    }


}


