package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="holonomicDrive Any Angle", group="Testing")
public class holonomicDrive_0_5 extends LinearOpMode
{
    Bogg robot;

    Gamepad g1;

    double initialAngle = Math.PI;

    @Override
    public void runOpMode()
    {

        robot = new Bogg(hardwareMap, telemetry, Bogg.Name.Bogg);
        robot.driveEngine.driveAtAngle(initialAngle);
        g1 = gamepad1;
        waitForStart();

        while (opModeIsActive())
        {
            if(!robot.manualRotate(g1.right_stick_button, g1.right_stick_x)) //if we're not rotating
            {
                robot.driveEngine.driveAtAngle(initialAngle - robot.sensors.getImuHeading());
                robot.manualDrive(g1.left_stick_button, g1.left_stick_x, g1.left_stick_y);
            }

            if(g1.dpad_down)
                robot.setBrake(Bogg.Direction.On);
            else if(g1.dpad_up)
                robot.setBrake(Bogg.Direction.Off);


            if(g1.left_bumper)
                robot.dropMarker(Bogg.Direction.Left);
            else if(g1.right_bumper)
                robot.dropMarker(Bogg.Direction.Right);


            robot.manualLift(g1.y, g1.a);


            telemetry.update();
            robot.update();
            idle();
        }
    }
}
