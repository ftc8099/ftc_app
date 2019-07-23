package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Mode;

import java.util.Arrays;

public abstract class AutonomousOpmode extends LinearOpMode
{
    Auto auto;

    private Mode[] order = {};

    void setOrder(Mode ... order){
        this.order = order;
    }

    public void runOpMode()
    {
        auto = new Auto(hardwareMap, telemetry);
        Mode action = Mode.Drop;

        waitForStart();

        auto.timer.reset();
        while (opModeIsActive())
        {
            Mode nextAction = auto.run(action);
            switch (nextAction)
            {
                case Same:
                    break;
                case Next:
                    try{
                        action = order[Arrays.binarySearch(order, action) + 1];
                    } catch(ArrayIndexOutOfBoundsException e){
                        action = Mode.Stop;
                    }
                    break;
                default:
                    action = nextAction;
            }

            // Display the current values
            telemetry.addData("mode:", action);
            auto.update();
            idle();
        }
        auto.stop();
    }
}
