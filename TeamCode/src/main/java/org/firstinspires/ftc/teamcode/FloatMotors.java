package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="  float motors", group="Testing")
<<<<<<< HEAD
public class FloatMotors extends LinearOpMode
=======
class floatMotors extends LinearOpMode
>>>>>>> alternate-smoothing-branch
{
    Bogg robot;

    @Override
    public void runOpMode()
    {
        robot = Bogg.determineRobot(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            robot.floatMotors();

            robot.update();
            idle();
        }
    }
}

