package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="4: Competition", group = "Competition")
public class autonomousDrive4_DropPlacePark extends AutonomousOpmode
{
    @Override
    public void runOpMode(){
        setOrder(Auto.Mode.Drop,
                Auto.Mode.Slide1,
                Auto.Mode.Slide2,
                Auto.Mode.WaitToDetectPicture,
                Auto.Mode.MoveToDepot,
                Auto.Mode.DropMarker,
                Auto.Mode.MoveToCrater,
                Auto.Mode.Stop);

        super.runOpMode();
    }
}
