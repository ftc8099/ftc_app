package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="holonomicDrive Normal", group="Testing")
public class holonomicDrive_0_1 extends LinearOpMode
{
    Bogg robot;

    Gamepad g1;

    @Override
    public void runOpMode()
    {

        robot = new Bogg(hardwareMap, gamepad1, telemetry);
        robot.driveEngine.driveAtAngle(Math.PI);
        g1 = gamepad1;
        waitForStart();

        while (opModeIsActive())
        {
            if(!robot.dPadOrbit(g1.dpad_left, g1.dpad_right)) //if we're not orbiting
            {
                if(!robot.manualRotate(g1.right_stick_button, g1.right_stick_x)) //if we're not rotating
                {
                    robot.manualDrive(g1.left_stick_button, g1.left_stick_x, g1.left_stick_y);
                }
            }

            robot.manualBrake(g1.dpad_down, g1.dpad_up);


            if(g1.left_bumper)
                robot.dropMarker(Bogg.Direction.Left);
            else if(g1.right_bumper)
                robot.dropMarker(Bogg.Direction.Right);


            robot.manualLift(g1.y, g1.a);

            // Display the current value

            telemetry.update();
            idle();
        }
    }
}

