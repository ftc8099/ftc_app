package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autoTestRotateAndTranslate", group = "Testing")
public class autoTestRotateAndTranslate extends LinearOpMode {
    Bogg robot;
    public void runOpMode()
    {
        robot = new Bogg(hardwareMap, telemetry, Bogg.Name.Bogg);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive())
        {
            if(robot.driveEngine.moveOnPath("hi", DriveEngine.Positioning.Absolute,
                    new double[]{0,12,Math.PI}))
                if(timer.seconds() > 6)
                    robot.driveEngine.moveOnPath(DriveEngine.Positioning.Absolute,
                        new double[]{0,0, Math.PI});

            robot.update();
            idle();
        }
    }
}
