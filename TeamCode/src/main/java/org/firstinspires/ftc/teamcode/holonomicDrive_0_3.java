package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="holonomicDrive Experimental", group="Testing")
public class holonomicDrive_0_3 extends LinearOpMode
{
    Bogg robot;
    double time;
    DcMotor hex;

    @Override
    public void runOpMode()
    {
        hex = hardwareMap.dcMotor.get("hex");
        robot = new Bogg(hardwareMap, gamepad1, telemetry);
        waitForStart();
        double x = .6;
        boolean changingAngle = false;
        double initialAngle = 0;

        while (opModeIsActive())
        {

            if(gamepad1.right_stick_x != 0 )
            {
                robot.manualRotate();
            }
            else
            {
                robot.manualDrive();
            }
            if(gamepad1.dpad_up && x < .7)
            {
                x+=.01;
            }

            if(gamepad1.dpad_down && x > .4)
            {
                x-=.01;
            }
            robot.setBrake(x);

            double x2 = gamepad2.left_stick_x;
            double y2 = gamepad2.left_stick_y;
            double magnitude = Math.sqrt(x2*x2+y2*y2);


            if(changingAngle)
            {
                if(magnitude > .90)
                {
                    double deltaAngle = Math.atan2(y2, x2) - initialAngle;
                    robot.driveEngine.driveAtAngle(initialAngle + deltaAngle);
                }
                else
                {
                    changingAngle = false;
                }
            }
            else
            {
                if(magnitude > .90)
                {
                    changingAngle = true;
                    initialAngle = Math.atan2(y2, x2);
                }
            }
            setHexAngle(robot.driveEngine.theta);

            robot.manualLift();

            // Display the current value
            telemetry.addData("Servo x", x);
            telemetry.addData("touch", robot.sensors.touchBottom.isPressed());
            telemetry.addData("fixed distance", robot.sensors.dFixed.getDistance(DistanceUnit.INCH));
            telemetry.addData("mobile distance", robot.sensors.dMobile.getDistance(DistanceUnit.INCH));
            if(robot.camera != null)
            {
                telemetry.addData("camera x, y", robot.camera.getLocation() == null ? "N/A" : robot.camera.getLocation());
                telemetry.addData("camera orientation", robot.camera.getHeading() == null ? "N/A" : robot.camera.getHeading());
            }

            telemetry.update();
            idle();
        }
    }

    double ticksPerRev = 1120;
    void setHexAngle(double angle)
    {
        double revs = angle/ (2* Math.PI) + 0;
        hex.setTargetPosition((int) (revs * ticksPerRev));
    }
}

