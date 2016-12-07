package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class DriveEngine
{
    public enum engineMode
    {
        defaultMode,
        pusherMode,
        levelMode, //This is a new drive mode to work on
        directMode,
    }

    private ElapsedTime timer = null;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    Gamepad gamepad = null;

    engineMode mode = engineMode.defaultMode;

    DcMotor.Direction leftDirection = DcMotor.Direction.REVERSE;
    DcMotor.Direction rightDirection = DcMotor.Direction.FORWARD;
    double leftPower = 0;  //[0.0, 1.0]
    double rightPower = 0; //[0.0, 1.0]

    double inchesBetweenMotors = 14;

    //Variables for pusherMode
    static final int SAMPLE_SIZE = 15;
    List<Float> rightJoyStickValues = new ArrayList();
    List<Float> leftJoyStickValues = new ArrayList();


    public DriveEngine(engineMode mode, HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.mode = mode;
        this.gamepad = gamepad;

        leftMotor  = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        leftMotor.setDirection(leftDirection);
        rightMotor.setDirection(rightDirection);
        timer = new ElapsedTime();

        for(int i=0; i<SAMPLE_SIZE; i++)
        {
            rightJoyStickValues.add(0f);
            leftJoyStickValues.add(0f);
        }
    }

    public Double getLeftPower()
    {
        return leftPower;
    }

    public Double getRightPower()
    {
        return rightPower;
    }

    public void setEngineMode(engineMode mode)
    {
        this.mode = mode;
    }

    public void invertDirection()
    {
        if (leftDirection == DcMotor.Direction.FORWARD)
            leftDirection = DcMotor.Direction.REVERSE;
        else
            leftDirection = DcMotor.Direction.FORWARD;

        if (rightDirection == DcMotor.Direction.FORWARD)
            rightDirection = DcMotor.Direction.REVERSE;
        else
            rightDirection = DcMotor.Direction.FORWARD;
    }

    private void resetJoyStickSamples(float value)
    {
        for(int i=0; i<SAMPLE_SIZE; i++)
        {
            rightJoyStickValues.set(i, value);
            leftJoyStickValues.set(i, value);
        }
    }

    private void takeJoyStickSample()
    {

        //Copied Preston's code here...
        //Need to review the logic to use absolute power.
        //What we really need to know is when we change
        //direction.  The values of the stick go from -1 to 1...

        if(Math.abs(gamepad.right_stick_y) < Math.abs(rightPower))
            resetJoyStickSamples(gamepad.right_stick_y);

        if(Math.abs(gamepad.left_stick_y) < Math.abs(leftPower))
            resetJoyStickSamples(gamepad.left_stick_y);


        rightJoyStickValues.remove(0);
        rightJoyStickValues.add(gamepad.right_stick_y);

        leftJoyStickValues.remove(0);
        leftJoyStickValues.add(gamepad.left_stick_y);
    }

    private void setEngineToAveragePower()
    {
        takeJoyStickSample();

        float sumRight = 0;
        float sumLeft = 0;
        for(int i=0; i<SAMPLE_SIZE; i++)
        {
            sumRight += rightJoyStickValues.get(i);
            sumLeft += leftJoyStickValues.get(i);
        }
        rightPower = sumRight/SAMPLE_SIZE;
        leftPower = sumLeft/SAMPLE_SIZE;
    }

    public void setEngineToGamePad()
    {
        this.rightPower = gamepad.right_stick_y;
        this.leftPower = gamepad.left_stick_y;
    }

    public void setEngineToPower(double rPower, double lPower)
    {
        rightPower = rPower;
        leftPower = lPower;
    }

    public void setEngineToLevel()
    {
        /*
        This is a new drive mode.  The objective is to try
        another way to gently increase the power as to not loose control.
        Use predefined power levels and increase the power of a short
        period of time.
        Will need constant powers, constant time intervals, and a timer.
         */
    }

    public void turn(double degree)
    {
        /*
        For Autonomous mode we will need to be able to turn.
        This function will take the degree of the turn we wish to make.
        A good first assumption is that only one wheel is turning at a time.
        A sloping turn will have both wheels moving at different speeds...
         */
    }

    public void drive(double duration, double rPower, double lPower)
    {
        switch (mode)
        {
            case pusherMode:
                setEngineToAveragePower();
                break;

            case directMode:
                setEngineToPower(rPower, lPower);
                break;

            case defaultMode:
                setEngineToGamePad();
                break;

            default:
                leftPower = rightPower = 0f;
        }

        if(duration != 0)
            timer.reset();

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        while(duration != 0 && timer.time() < duration);
    }

    public void drive()
    {
        drive(0d, 0d, 0d);
    }

    public void drive(double power)
    {
        drive(0d, power, power);
    }

    public void drive(double duration, double power)
    {
        drive(duration, power, power);
    }

    public void stop()
    {
        leftPower = rightPower = 0f;
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}
