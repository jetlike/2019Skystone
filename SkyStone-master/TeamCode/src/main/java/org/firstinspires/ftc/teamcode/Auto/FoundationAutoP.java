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

@Autonomous(group = "godly", name = "FoundationAutoP")
public class FoundationAutoP extends Methods {


    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {


        ready();

        // wait for start button.

        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        MoveInch(1, 12.5);
        telemetry.addData("Running MoveInch1:", "complete");
        telemetry.update();
        sleep(1000);

        FoundationClamp(0.5, 0.5);
        telemetry.addData("Running Clamp1:", "complete");
        telemetry.update();
        sleep(1000);

        MoveInch(-.5, 26);
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

        MoveInch(1, 7);
        telemetry.addData("Running MoveInch3:", "complete");
        telemetry.update();
        sleep(1000);

        Strafe(-1, 6);
        telemetry.addData("Running strafe2:", "complete");
        telemetry.update();
        sleep(1000);

        MoveInch(-1, 7);
        telemetry.addData("Running MoveInch4:", "complete");
        telemetry.update();
        sleep(1000);

        Strafe(1, 15);
        telemetry.addData("Running strafe3:", "complete");
        telemetry.update();
        sleep(1000);


    }


}


