package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="5: Ground Competition", group = "Testing")
public class autonomousDrive5_SortPlacePark extends LinearOpMode
{
    Auto auto;
    Auto.Mode action;

    @Override
    public void runOpMode()
    {
        auto = new Auto(hardwareMap, telemetry);
        action = Auto.Mode.Drop;
//        auto.robot.setBrake(Bogg.Direction.On);
//        auto.robot.dropMarker(Bogg.Direction.Up);
        waitForStart();

        auto.timer.reset();
        while (opModeIsActive())
        {
            switch(action)
            {
                case Drop:
                    action = auto.drop();
                    break;

                    //Can't reliably see all minerals after slide...
                case LookForMinerals:
                    if(auto.camera.canUseTFOD) {
                        action = auto.lookForMinerals();
                        break;
                    }

                case Slide1:
                    action = auto.slide1();
                    break;

                case PushGold:
                    action = auto.pushGold();
                    break;

                case Slide2:
                    action = auto.slide2();
                    break;
                case WaitToDetectPicture:
                    action = auto.waitToDetectPicture();
                    break;

                case MoveToDepot:
                    action = auto.moveToDepot();
                    break;

                case DropMarker:
                    action = auto.dropMarker();
                    break;
//
//                case MoveToCrater:
//                    action = auto.moveToCrater();
//                    break;
                default:
                    action = auto.stop();
            }
            telemetry.addData("TouchTop: ", auto.robot.sensors.touchTopIsPressed());
            telemetry.addData("TouchBottom: ", auto.robot.sensors.touchBottomIsPressed());
            // Display the current values
            telemetry.addData("mode:", action);
            auto.update();
            idle();
        }
        auto.stop();
    }
}
