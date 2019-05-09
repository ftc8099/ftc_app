package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="holonomicDrive Competition", group="Competition")
public class holonomicDrive_0_2 extends LinearOpMode
{
    Bogg robot;


    @Override
    public void runOpMode()
    {
        robot = new Bogg(hardwareMap, telemetry, Bogg.Name.Bogg);
        telemetry.addLine("ready");
        waitForStart();

        if(robot.name == Bogg.Name.Bogg)
            robot.endEffector.pivot.setPower(1);
        Gamepad g1 = gamepad1;
        Gamepad g2 = gamepad2;


        while (opModeIsActive())
        {
            if(g1.timestamp > g2.timestamp)
                telemetry.addLine("Using gamepad1");
            else if(g1.timestamp < g2.timestamp)
                telemetry.addLine("Using gamepad2");
            else
                telemetry.addLine("Know which gamepad you're using");


            //Servos
            if(g1.dpad_down)
                robot.setBrake(Bogg.Direction.On);
            else if(g1.dpad_up)
                robot.setBrake(Bogg.Direction.Off);


            if(g1.right_bumper)
                robot.dropMarker(Bogg.Direction.Down);
            else
                robot.dropMarker(Bogg.Direction.Up);


            //Lift
            if(robot.manualLift(g1.y, g1.a))
                robot.driveEngine.stop();

            //Drive angle
            if(g1.x)
                robot.driveEngine.resetFieldHeadingToRobotHeading();


            //Arm and drive manual
            robot.manualDriveFixedForwardAutoCorrect(
                    g1.left_stick_button,
                    g1.left_stick_x,
                    g1.left_stick_y,
                    g1.right_stick_x);


            //if the moveOnPaths have finished
            robot.driveEngine.checkpoints.clear();


            if(robot.name == Bogg.Name.Bogg)
                if(!robot.endEffector.extend(-g2.left_stick_y)){
                    if(!robot.endEffector.extend(g2.right_stick_y))
                        robot.endEffector.contract.setPower(0);
                }


            if(g1.back)
                break;

            // Display the current values
            telemetry.addLine("'Pressing A must move the arm down/robot up.'");
            telemetry.addLine("Set brake: d-down. Remove brake: d-up.");

            robot.update();
            idle();
        }
    }
}

