package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="4: Competition", group = "Testing")
public class autonomousDrive4_DropPlacePark extends LinearOpMode
{
    Bogg robot;
    Auto auto;
    Auto.Mode action;


    @Override
    public void runOpMode()
    {
        robot = new Bogg(hardwareMap, telemetry);
        auto = new Auto(robot, hardwareMap, telemetry);
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
                case MoveToCrater:
                    action = auto.moveToCrater();
                default:
                    auto.stop();
            }

            // Display the current values
            telemetry.addData("mode:", action);
            telemetry.update();
            robot.update();
            idle();
        }
        auto.stop();
    }
}

