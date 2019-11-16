package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CompTeleOp", group="Android")
public class CompTeleOp extends LinearOpMode
{
    Bogg robot;

    @Override
    public void runOpMode()
    {
        robot = new Bogg(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive())
        {
            robot.manualDrive(gamepad1.left_stick_x*0.75, gamepad1.left_stick_y*0.75, -0.5*gamepad1.right_stick_x);
            robot.moveArm(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_left, gamepad1.dpad_right);
            robot.getBlock(gamepad1.a, gamepad1.b);
            telemetry.update();
            idle();
        }
    }
}

