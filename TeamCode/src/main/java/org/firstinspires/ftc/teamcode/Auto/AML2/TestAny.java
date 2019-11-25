package org.firstinspires.ftc.teamcode.Auto.AML2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Methods.AML2Methods;

import java.util.List;

@Autonomous(name = "testany", group = "test")
public class TestAny extends AML2Methods {
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {


    ready();


    waitForStart();

    MoveInchP(1, 24, 5);
    }
}
