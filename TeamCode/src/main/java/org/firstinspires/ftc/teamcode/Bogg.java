package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bogg
{
    DriveEngine driveEngine;
    DcMotor lift;
    Sensors sensors;
    ElapsedTime timer;
    Servo brake;
    Servo drop;
    Name name;

    static double averageClockTime = 0;
    private double liftAve = 0;

    private boolean rotating = false;


    Telemetry telemetry;

    enum Direction
    {
        Left,
        Straight,
        Right,
        Down,
        Up,
        On,
        Off
    }

    enum Name
    {
        Bogg,
        MiniBogg,
        Fauxbot,
        Fakebot
    }

    public Bogg(HardwareMap hardwareMap, Telemetry telemetry, Name whichRobot)
    {
        this.telemetry = telemetry;
        this.name = whichRobot;
        timer = new ElapsedTime();

        switch (whichRobot)
        {
            case Bogg:
                lift  = hardwareMap.dcMotor.get("lift");
                lift.setDirection(DcMotorSimple.Direction.FORWARD);
                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                brake = hardwareMap.servo.get("brake");
                drop = hardwareMap.servo.get("drop");
                break;

            case MiniBogg:
                lift  = hardwareMap.dcMotor.get("lift");
                lift.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case Fauxbot:
                driveEngine.setInitialAngle(-Math.PI / 2);
                break;

            case Fakebot:
                driveEngine = new FakeDriveEngine(telemetry);
                break;
        }

        sensors = new Sensors(hardwareMap, whichRobot);

        switch (whichRobot)
        {
            case Bogg:
                driveEngine = new OmniWheelDriveEngine(hardwareMap, telemetry, sensors, 3);
                break;

            case MiniBogg:
                driveEngine = new OmniWheelDriveEngine(hardwareMap, telemetry, sensors, 3);
                driveEngine.mP *= 5;
                OmniWheelDriveEngine.ticksPerRev = 290;
                OmniWheelDriveEngine.effectiveWheelDiameter = 3.5;
                break;

            case Fauxbot:
                driveEngine = new OmniWheelDriveEngine(hardwareMap, telemetry, sensors, 2);
                break;
        }
    }

    /**
     * Returns the appropriate Bogg based on the hardware map.
     * Allows any program to be used on any robot, or no robot at all.
     * No robot will return a Fakebot with a FakeDriveEngine.
     * @return the Bogg you are looking for
     */
    static Bogg determineRobot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        Bogg robot;
        try {
            robot = new Bogg(hardwareMap, telemetry, Name.Bogg);
        } catch (Exception e) {
            telemetry.addLine(e.toString());
            try {
                robot = new Bogg(hardwareMap, telemetry, Name.MiniBogg);
            } catch (Exception e1) {
                telemetry.addLine(e1.toString());
                try {
                    robot = new Bogg(hardwareMap, telemetry, Name.Fauxbot);
                } catch (Exception e2) {
                    telemetry.addLine(e2.toString());
                    robot = new Bogg(hardwareMap, telemetry, Name.Fakebot);
                }
            }
        }
        telemetry.addData("Name", robot.name);
        return robot;
    }


    /**
     * This method finds an equilibrium lifting point using an exponential average
     * @param l: How much to lift
     * @return: The smoothed lifting value
     */
    private double smoothLift(double l)
    {
        double liftAlpha = .12;
        if(l* liftAve < 0 || l == 0)
            liftAve = 0;
        else if(l == -.02)
        {
            liftAve = liftAlpha * l + (1- liftAlpha) * liftAve;
        }
        else
            liftAve = liftAlpha /3 * l + (1- liftAlpha /3) * liftAve;
        return liftAve;
    }

    /**
     * This method finds an equilibrium point based on limit sensor data and smoothing
     * @param up is whether the up button is pushed
     * @param down is whether the down button is pushed
     * @return if pulling arm down
     */
    boolean manualLift(boolean up, boolean down)
    {
        if(name == Name.Bogg)
        if(up && !sensors.touchTopIsPressed())
        {
            lift.setPower(smoothLift(1));
        }
        else if(down)
        {
            if (sensors.touchBottomIsPressed())
            {
                lift.setPower(smoothLift(-.02));
                return true;
            } else {
                lift.setPower(smoothLift(-1));
                return true;
            }
        }
        else
            lift.setPower(smoothLift(0));

        return false;
    }

    /**
     * Assigns power to the lift motor based on limit sensor data
     * @param power to run the lift at
     */
    void lift(double power)
    {
        if(name == Name.Bogg)
        if(power > 0  && !sensors.touchTopIsPressed())
            lift.setPower(smoothLift(power));
        else if(power < 0 && !sensors.touchBottomIsPressed())
            lift.setPower(smoothLift(power));
        else
            lift.setPower(smoothLift(0));
    }

    /**
     * Sets the brake
     * @param direction to set the brake
     */
    void setBrake(Direction direction)
    {
        if(name == Name.Bogg)
        switch (d) {
            case On:
                brake.setPosition(.73);
                break;
            case Off:
                brake.setPosition(.60);
        }
    }

    /**
     * Sets the marker-dropper position
     * @param direction to move the marker-dropper
     */
    void dropMarker(Direction direction)
    {
        if(name == Name.Bogg)
        switch (direction)
        {
            case Down:
                drop.setPosition(.3);
                break;
            case Up:
            default:
                drop.setPosition(0);
                break;
        }
    }

    /**
     * This method enables pretty direct control over the robot, no frills.
     * @param op: Overpowered maxes at least one motor when you press the joystick far enough.
     * @param x: Velocity in the x direction
     * @param y: Velocity in the y direction
     * @param spin: Rotational velocity
     */
    void manualDrive(boolean op, double x, double y, double spin)
    {
        telemetry.addData("gamepad x", x);
        telemetry.addData("gamepad y", y);
        telemetry.addData("gamepad spin", spin);
        telemetry.addLine("Note: y and spin are negated");
        //The gamepad naturally gives negative y values.
        //Spin is naturally counterclockwise, but negating it is easier for people to control.

        driveEngine.drive(op, x, -y, -spin);
    }

    ElapsedTime spinTimer = new ElapsedTime();


    /**
     * This method automatically corrects for incidental rotation.
     * It also orients the gamepad direction to the field.
     * Note: in order to reset the field heading, you will need to call
     *      driveEngine.resetFieldHeadingToRobotHeading()
     * @param op: Overpowered maxes at least one motor when you press the joystick far enough.
     * @param x: Velocity in the x direction
     * @param y: Velocity in the y direction
     * @param spin: Rotational velocity
     */
    void manualDriveFixedForwardAutoCorrect(boolean op, double x, double y, double spin)
    {
        manualDriveAutoCorrect(op, x, y, spin);

        driveEngine.orientRobotDirectionToField();

        telemetry.addData("gamepad x", x);
        telemetry.addData("gamepad y", y);
        telemetry.addData("gamepad spin", spin);
        telemetry.addLine("Note: y and spin are negated");
    }


    /**
     *This method automatically corrects for incidental rotation.
     * @param op: Overpowered maxes at least one motor when you press the joystick far enough.
     * @param x: Velocity in the x direction
     * @param y: Velocity in the y direction
     * @param spin: Rotational velocity
     */
    void manualDriveAutoCorrect(boolean op, double x, double y, double spin)
    {
        //While spinning, no autocorrection
        if(spin != 0)
            spinTimer.reset();
        if(spinTimer.seconds() < 1) {
            driveEngine.resetForward();
            driveEngine.drive(0, op? 1:2.5, op,true,true,
                    x, -y, -spin);
        }
        //After a second for the sensor to get up to date, we start to correct for rotation.
        else {
            driveEngine.drive(0, op? 1:2.5, op,true,false,
                    x, -y, driveEngine.angularVelocityNeededToFaceForward());
        }


        telemetry.addData("gamepad x", x);
        telemetry.addData("gamepad y", y);
        telemetry.addData("gamepad spin", spin);
        telemetry.addLine("Note: y and spin are negated");
    }

    /**
     * There's some kind of internal brake on the motors.
     * This method releases that brake on motors that the robot controls.
     */
    void floatMotors()
    {
        driveEngine.floatMotors();
        if(name == Name.Bogg)
        {
            endEffector.floatMotors();
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("lift position", lift.getCurrentPosition());
        }
    }


    private double lastTime = 0;
    /**
     * Should only be called once per loop
     * Updates the average time for one loop
     * Updates the driveEngine, telemetry, and sensors.
     */
    void update()
    {
        double t = timer.seconds();
        double clockTime = t - lastTime;
        lastTime = t;
        if(clockTime == t) //Ensures that the first time through doesn't affect the average.
            return;

        averageClockTime = (clockTime * .02 + averageClockTime * .98); //exponential average
        //The average is needed because the clocktime can change over time.

        driveEngine.update();
        sensors.update();
        telemetry.update();
        telemetry.addData("currentClockTime", clockTime);
        telemetry.addData("averageClockTime", averageClockTime);
    }

    /**
     * @param seconds: number of seconds to reach 95% of target value
     * @return the alpha value to use in an exponential averaging system.
     */
    static double getAlpha(double seconds)
    {
        if(seconds == 0)
            return 1;
        return 1 - Math.pow(.05, averageClockTime/seconds); //reaches 95% in this many seconds
    }
}
