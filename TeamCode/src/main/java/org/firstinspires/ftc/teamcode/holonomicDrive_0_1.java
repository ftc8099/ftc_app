package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name="holonomicDrive Normal", group="Testing")
public class holonomicDrive_0_1 extends LinearOpMode
{
    Bogg robot;

    @Override
    public void runOpMode()
    {
        robot = Bogg.determineRobot(hardwareMap, telemetry);
        //Sets the front of the robot.
        robot.driveEngine.setInitialAngle(Math.PI);
        Gamepad g1 = gamepad1;
        waitForStart();

        while (opModeIsActive())
        {
            robot.manualDrive(g1.left_stick_button, g1.left_stick_x, g1.left_stick_y,g1.right_stick_x);

            if(g1.dpad_down)
                robot.setBrake(Bogg.Direction.On);
            else if(g1.dpad_up)
                robot.setBrake(Bogg.Direction.Off);


            if(g1.left_bumper)
                robot.dropMarker(Bogg.Direction.Left);
            else if(g1.right_bumper)
                robot.dropMarker(Bogg.Direction.Right);


            robot.manualLift(g1.y, g1.a);


            robot.update();
            idle();
        }
    }
}

