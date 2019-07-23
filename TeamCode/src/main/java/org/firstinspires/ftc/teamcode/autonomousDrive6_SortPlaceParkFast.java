package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Lander Fast Competition", group = "Competition")
public class autonomousDrive6_SortPlaceParkFast extends AutonomousOpmode
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

