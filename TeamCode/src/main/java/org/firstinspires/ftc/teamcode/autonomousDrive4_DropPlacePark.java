package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="4: Competition", group = "Competition")
public class autonomousDrive4_DropPlacePark extends LinearOpMode
{
    Auto auto;
    Auto.Mode action;


    @Override
    public void runOpMode()
    {
        auto = new Auto(hardwareMap, telemetry);
        action = Auto.Mode.Drop;
        waitForStart();

        while (opModeIsActive())
        {
            switch(action)
            {
                case Drop:
                    action = auto.drop();
                    break;
                case LookForMinerals:
                case Slide1:
                    action = auto.slide1();
                    break;
                case PushGold:
                case Slide2:
                    action = auto.slide2();
                    break;
                case TurnByCamera:
                    action = auto.turnByCamera();
                    break;
                case MoveToDepot:
                    action = auto.moveToDepot();
                    break;
                case DropMarker:
                    action = auto.dropMarker();
                    break;
                case MoveToCrater:
                    action = auto.moveToCrater();
                    break;
                default:
                    action = auto.stop();
            }

            // Display the current values
            telemetry.addData("mode:", action);
            auto.update();
            idle();
        }
        auto.stop();
    }
}

