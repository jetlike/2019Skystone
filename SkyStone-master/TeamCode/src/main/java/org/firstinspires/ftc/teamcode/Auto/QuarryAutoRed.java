package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(group = "godlyQuarry", name = "QuarryAutoRed")
public class QuarryAutoRed extends Methods {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        ready();


        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {

            liftPower(299, 1); //have to do it here, because for some whack reason, it doesn't work in the ready method


            MoveInch(.4, 27.5);
            sleep(400);

            GrabBrick(.7);
            sleep(400);

            liftPower(10, 1);
            sleep(400);

            MoveInch(-1, 9.3);
            sleep(400);

            Strafe(.5, 38);
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
        }
    }


}