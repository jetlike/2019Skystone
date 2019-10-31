package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(group = "godly", name = "QuarryAutoP")
public class QuarryAutoP extends FoundationAutoP {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        ready();

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        MoveInch(.75, 7);
        sleep(1000);

        GrabBrick(0.85);
        sleep(1500);

        MoveInch(-0.75, 2);
        sleep(1000);

        rotate(90, .7);
        sleep(1000);

        MoveInch(.6,12.5);
        sleep(1000);

        GrabBrick(0.1);
        sleep(1000);

        MoveInch(-0.6, 9 );
        sleep(1000);

        rotate(-90, 0.7);
        sleep(1000);

        MoveInch(.75, 2);
        sleep(1000);

        GrabBrick(0.85);
        sleep(1500);

        MoveInch(-0.75, 2);
        sleep(1000);

        rotate(90, 0.7);
        sleep(1000);

        MoveInch(0.6, 9);
        sleep(1000);
    }





}


