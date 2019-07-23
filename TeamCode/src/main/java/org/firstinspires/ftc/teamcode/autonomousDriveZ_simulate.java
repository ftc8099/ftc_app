package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name="Z: Fake", group = "Testing")
public class autonomousDriveZ_simulate extends AutonomousOpmode
{
    @Override
    public void runOpMode()
    {
        gamepad1 = new Gamepad();
        gamepad1.setGamepadId(5);

        setOrder(Auto.Mode.Drop,
                Auto.Mode.Slide1,
                Auto.Mode.Slide2,
                Auto.Mode.MoveToDepot,
                Auto.Mode.MoveToCrater,
                Auto.Mode.Stop);

        super.runOpMode();
    }
}

