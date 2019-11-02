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

@Autonomous(group = "godly", name = "Park")
public class Park extends Methods {


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {


        ready();

        // wait for start button.

        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        MoveInch(.6, 12);
        telemetry.addData("moving forward to park:", "autocomplete");
        telemetry.update();

    }
}