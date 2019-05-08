package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="test servos", group="Testing")
public class test_servos extends LinearOpMode
{
    Bogg robot;

    Gamepad g1;

    //Servos go from 0 to 1.
    //Seconds to go from one side to the other
    double seconds = 6;

    @Override
    public void runOpMode()
    {
        robot = Bogg.determineRobot(hardwareMap, telemetry);
        g1 = gamepad1;

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addLine(robot.name.toString());

            adjustServoPosition(robot.endEffector.pinch, "pinch", g1.left_stick_y);

            adjustServoPosition(robot.endEffector.swing, "swing", g1.right_stick_y);

            adjustServoPosition(robot.drop, "drop", g1.y, g1.a);

            adjustServoPosition(robot.brake, "brake", g1.left_stick_button, g1.right_stick_button);

            sleep(50);
            robot.update();
            idle();
        }
    }

    private void adjustServoPosition(Servo servo, String name, double speed)
    {
        if(g1.left_stick_y != 0)
        {
            servo.setPosition(Range.clip(servo.getPosition() + speed * Bogg.averageClockTime / seconds,0,1));
        }
        telemetry.addData(name, servo.getPosition());
    }

    private void adjustServoPosition(Servo servo, String name, boolean positive, boolean negative)
    {
        if(positive || negative)
        {
            servo.setPosition(Range.clip(servo.getPosition() + (positive? 1:-1) * Bogg.averageClockTime / seconds, 0,1));
        }
        telemetry.addData(name, servo.getPosition());
    }
}

