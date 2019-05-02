package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="  float motors", group="Testing")
<<<<<<< HEAD
<<<<<<< HEAD
public class FloatMotors extends LinearOpMode
=======
class floatMotors extends LinearOpMode
>>>>>>> alternate-smoothing-branch
=======
public class floatMotors extends LinearOpMode
>>>>>>> f6d25de99d4c34681c84a89e562aa08f6a6f862b
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

