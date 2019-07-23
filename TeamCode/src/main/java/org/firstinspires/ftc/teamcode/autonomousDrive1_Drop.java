package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Start on Lander Competition", group = "Testing")
public class autonomousDrive1_Drop extends AutonomousOpmode
{
    @Override
    public void runOpMode() {
        setOrder(Auto.Mode.Drop,
                Auto.Mode.LookForMinerals,
                Auto.Mode.Slide1,
                Auto.Mode.PushGold,
                Auto.Mode.Slide2,
                Auto.Mode.WaitToDetectPicture,
                Auto.Mode.MoveToDepot,
                Auto.Mode.DropMarker,
                Auto.Mode.Stop);

        super.runOpMode();
    }
}

