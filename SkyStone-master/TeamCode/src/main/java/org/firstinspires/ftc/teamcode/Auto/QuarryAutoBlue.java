package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(group = "godly", name = "QuarryAutoBlue")
public class QuarryAutoBlue extends Methods {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        ready();

        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();


        while (!isStopRequested() && opModeIsActive()) {
            MoveInch(.5, 25);
            telemetry.addData("Moving to get block:", "complete");
            telemetry.update();
            sleep(1000);

            GrabBrick(0.85);
            telemetry.addData("Grabbing the block:", "complete");
            telemetry.update();
            sleep(1000);

            liftPower(-586, 1);
            telemetry.addData("lifting lift a little", "complete");
            telemetry.update();
            sleep(1000);

            MoveInch(-0.75, 4);
            telemetry.addData("Moving backward to deliver", "complete");
            telemetry.update();
            sleep(1000);

            Strafe(-.5, 32);
            telemetry.addData("strafing to deliver", "complete");
            telemetry.update();
            sleep(1000);

            GrabBrick(0.6);
            telemetry.addData("Delivering", "complete");
            telemetry.update();
            sleep(1000);

            Strafe(0.5, 46);
            telemetry.addData("moving back for 2nd one", "complete");
            telemetry.update();
            sleep(1000);

            liftPower(-200, 0.4);
            telemetry.addData("putting lift down:", "complete");
            telemetry.update();
            sleep(1000);

            MoveInch(0.5, 4);
            telemetry.addData("moving in for 2nd one", "complete");
            telemetry.update();
            sleep(1000);

            GrabBrick(0.85);
            telemetry.addData("grabbing brick", "complete");
            telemetry.update();
            sleep(1000);

            liftPower(-586, .4);
            telemetry.addData("moving back for 2nd one", "complete");
            telemetry.update();
            sleep(1000);

            MoveInch(-.75, 4);
            telemetry.addData("moving back", "complete");
            telemetry.update();
            sleep(1000);

            Strafe(-.5, 54);
            telemetry.addData("strafing to deliver, and parking", "done with auto");
            telemetry.update();
            sleep(1500);
            break;
        }
    }


}


