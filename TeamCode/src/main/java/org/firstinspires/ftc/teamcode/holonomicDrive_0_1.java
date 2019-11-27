package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name="holonomicDrive", group="")
public class holonomicDrive_0_1 extends LinearOpMode
{
    Bogg robot;

    @Override
    public void runOpMode()
    {
        waitForStart();

        while (opModeIsActive())
        {
            robot.manualDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            idle();
        }
    }
}

