package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="holonomicKids", group="Testing")
public class holonomicKids extends LinearOpMode
{
    Bogg robot;

    @Override
    public void runOpMode()
    {
        robot = new Bogg(hardwareMap, telemetry, Bogg.Name.MiniBogg);
        Gamepad g1 = gamepad1;

        waitForStart();

        while (opModeIsActive())
        {
            robot.manualDrive(g1.left_stick_button, g1.left_stick_x, g1.left_stick_y, g1.right_stick_x);

            // Display the current value

            robot.update();
            telemetry.update();
            idle();
        }
    }
}

