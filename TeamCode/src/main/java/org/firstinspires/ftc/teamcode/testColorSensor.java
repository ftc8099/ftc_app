package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="testColorSensor", group="Testing")
public class testColorSensor extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        MyColorSensor myColorSensor = new MyColorSensor(hardwareMap);
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("hue", myColorSensor.getHue());
            telemetry.addData("color", myColorSensor.getColor().name());
            telemetry.addData("typicalHue", myColorSensor.getColor().typicalHue);

            telemetry.update();
            idle();
        }
    }
}

