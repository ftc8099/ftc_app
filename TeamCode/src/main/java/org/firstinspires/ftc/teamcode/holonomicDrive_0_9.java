package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

@TeleOp(name="holonomicDrive Demonstration", group="Testing")
public class holonomicDrive_0_9 extends LinearOpMode
{
    Bogg robot;

    private boolean runToTouch = false;

    @Override
    public void runOpMode()
    {
        robot = Bogg.determineRobot(hardwareMap, telemetry);
        Gamepad g1 = gamepad1;
        ArrayList<double[]> blackBox = new ArrayList<>();
        waitForStart();

        while (opModeIsActive())
        {
            //Run to origin
            if(g1.a)
            {
                robot.driveEngine.moveOnPath(DriveEngine.Positioning.Absolute,
                        new double[]{0,0,0});
                blackBox.add(robot.driveEngine.blackValues);
            }

            //Run to touch
            else if(robot.screen.wasTouched() || runToTouch)
            {
                runToTouch = !robot.driveEngine.moveOnPath(DriveEngine.Positioning.Absolute,
                        robot.screen.getFieldCoordinates());
                blackBox.add(robot.driveEngine.blackValues);
            }

            //Retrace Path
            else if(g1.b && blackBox.size() > 0)
            {
                double[] values = blackBox.remove(blackBox.size() - 1);
                for (int i = 0; i < values.length; i++) {
                    values[i] *= -1;
                }
                robot.driveEngine.drive(values);
            }

            //Just Drive
            else {
                //Drive angle
                if(g1.x)
                    robot.driveEngine.resetFieldHeadingToRobotHeading();


                //Drive manual
                robot.manualDriveFixedForwardAutoCorrect(
                        g1.left_stick_button,
                        g1.left_stick_x,
                        g1.left_stick_y,
                        g1.right_stick_x);
                blackBox.add(robot.driveEngine.blackValues);
            }

            //Good practice: make the top class read like a gamepad map.
            if(g1.dpad_down)
                robot.setBrake(Bogg.Direction.On);
            else if(g1.dpad_up)
                robot.setBrake(Bogg.Direction.Off);

            if(g1.left_bumper)
                robot.dropMarker(Bogg.Direction.Left);
            else if(g1.right_bumper)
                robot.dropMarker(Bogg.Direction.Right);


            robot.manualLift(g1.y, g1.a);

            // Display the current value
            telemetry.addLine("'Pressing A must move the arm down/robot up.'");
            telemetry.addLine("Set brake: d-down. Remove brake: d-up.");

            robot.update();
            idle();
        }
    }
}

