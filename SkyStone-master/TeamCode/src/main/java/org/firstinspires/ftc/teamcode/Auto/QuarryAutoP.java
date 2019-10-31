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
        telemetry.addData("Moving to get block:", "complete");
        telemetry.update();
        sleep(1000);

        GrabBrick(0.85);
        telemetry.addData("Grabbing the block:", "complete");
        telemetry.update();
        sleep(1500);

        MoveInch(-0.75, 2);
        telemetry.addData("Moving backward to deliver", "complete");
        telemetry.update();
        sleep(1000);

        rotate(90, .7);
        telemetry.addData("Turning:", "complete");
        telemetry.update();
        sleep(1000);

        MoveInch(.6,12.5);
        telemetry.addData("Delivering", "complete");
        telemetry.update();
        sleep(1000);

        GrabBrick(0.1);
        telemetry.addData("dropping the block", "complete");
        telemetry.update();
        sleep(1000);

        MoveInch(-0.6, 9 );
        telemetry.addData("moving back", "complete");
        telemetry.update();
        sleep(1000);

        rotate(-90, 0.7);
        telemetry.addData("turning", "complete");
        telemetry.update();
        sleep(1000);

        MoveInch(.75, 2);
        telemetry.addData("moving to get block", "complete");
        telemetry.update();
        sleep(1000);

        GrabBrick(0.85);
        telemetry.addData("grab the block", "complete");
        telemetry.update();
        sleep(1500);

        MoveInch(-0.75, 2);
        telemetry.addData("move back to deliver", "complete");
        telemetry.update();
        sleep(1000);

        rotate(90, 0.7);
        telemetry.addData("turning to deliver", "complete");
        telemetry.update();
        sleep(1000);

        MoveInch(0.6, 9);
        telemetry.addData("delivering", "complete");
        telemetry.update();
        sleep(1000);
    }





}


