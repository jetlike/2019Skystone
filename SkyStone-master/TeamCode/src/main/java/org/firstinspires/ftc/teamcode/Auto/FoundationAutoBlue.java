package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(group = "godlyFoundation", name = "FoundationAutoBlue")
public class FoundationAutoBlue extends Methods {


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {


            ready();

            // wait for start button.


            waitForStart();
            telemetry.addData("Mode", "running");
            telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {

            MoveInch(.5, 28);
            telemetry.addData("Running MoveInch1:", "complete");
            telemetry.update();
            sleep(1000);

            FoundationClamp(0.5, 0.5);
            telemetry.addData("Running Clamp1:", "complete");
            telemetry.update();
            sleep(1000);

            MoveInch(-.5, 29);
            telemetry.addData("Running MoveInch2:", "complete");
            telemetry.update();
            sleep(1000);

            FoundationClamp(1, 0);
            telemetry.addData("Running Clamp2:", "complete");
            telemetry.update();
            sleep(1000);

            Strafe(.5, 22);
            telemetry.addData("Running strafe1:", "complete");
            telemetry.update();
            sleep(1000);

            MoveInch(.4, 35);
            telemetry.addData("Running MoveInch3:", "complete");
            telemetry.update();
            sleep(1000);

            Strafe(-0.5, 24);
            telemetry.addData("Running strafe2:", "complete");
            telemetry.update();
            sleep(1000);

            GrabBrick(0.85);
            telemetry.addData("Putting the clamp down", "complete");
            telemetry.update();
            sleep(1000);

            MoveInch(-.5, 25);
            telemetry.addData("Running MoveInch4:", "complete");
            telemetry.update();
            sleep(1000);

            Strafe(.5, 40);
            telemetry.addData("Running strafe3:", "complete");
            telemetry.update();
            sleep(1000);

            MoveInch(-0.2, 4);
            telemetry.addData("parking:", "done with auto");
            telemetry.update();
            break;
        }
    }
}


