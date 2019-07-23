package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.util.MyMath;


public class Auto {
    Bogg robot;
    Camera camera;
    Telemetry telemetry;
    ElapsedTime timer;
    private int iSP = -1; //initialSlopePositivity
    private int goldPosition = -1;

    Auto(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.robot = Bogg.determineRobot(hardwareMap, telemetry);
        robot.setBrake(Bogg.Direction.On);
        robot.dropMarker(Bogg.Direction.Up);
        robot.driveEngine.setInitialAngle(0);
        robot.driveEngine.setFieldAngle(-Math.PI /4);
        robot.driveEngine.setInitialPosition(6,6);
        this.telemetry = telemetry;
        camera = new Camera(hardwareMap, telemetry, false, true);
        telemetry.addLine("Camera Loaded");
        telemetry.addData("Gold Pos: ", camera.getGoldPosition());
        telemetry.addLine("Wait for start");
        telemetry.update();
        timer = new ElapsedTime();
    }

    interface Action{
        Mode run(Auto auto);
    }

    public enum Mode {
        Drop(Auto::drop),
        Slide1(Auto::slide1),
        LookForMinerals(Auto::lookForMinerals),
        PushGold(Auto::pushGold),
        Slide2(Auto::slide2),
        WaitToDetectPicture(Auto::waitToDetectPicture),
        MoveToDepot(Auto::moveToDepot),
        DropMarker(Auto::dropMarker),
        MoveToCrater(Auto::moveToCrater),
        Stop(Auto::stop),
        Same(Auto::stop),
        Next(Auto::stop);

        Action action;

        //Constructor for each Mode
        Mode(Action action) {
            this.action = action;
        }

    }

    Mode run(Mode mode){
        return mode.action.run(this);
    }

    //Robot starts in the air with touchBottom and touchTop both false
    //Pull robot up until touchBottom is true
    //Then move brake
    //Then lower robot until touchTop is true
    private boolean movingBrake = false;

    private Mode drop()
    {
        robot.driveEngine.disableDistanceTracking();
        if(!movingBrake)
        {
            timer.reset();
            movingBrake = true;
        }
        if(timer.seconds() < 2)
        {
            robot.lift(-1);
        }
        else if(timer.seconds() < 3)
        {
            robot.setBrake(Bogg.Direction.Off);
        }
        else if(!robot.sensors.touchTopIsPressed())
        {
            robot.lift(.2); //push up, which drops the robot
        }
        else if(robot.sensors.touchTopIsPressed()){
            timer.reset();
            robot.lift(0); //Turns motor off
            return Mode.Next;
        }
        return Mode.Same;
    }

    private Mode lookForMinerals()
    {
        telemetry.addLine("Looking for minerals");
        goldPosition = camera.getGoldPosition();
        if(goldPosition != -1) {
            return Mode.Next;
        }
        return Mode.Same;
    }

    private Mode slide1()
    {
        robot.driveEngine.enableDistanceTracking();
        if (robot.driveEngine.moveOnPath(DriveEngine.Positioning.Absolute,
                new double[]{3, 9}))
        {
            return Mode.Next;
        }
        return Mode.Same;
    }

    private Mode pushGold()
    {
        telemetry.addData("gold position", goldPosition);
        switch (goldPosition) {
            case 0:
                if (robot.driveEngine.moveOnPath(DriveEngine.Positioning.Absolute,
                        new double[]{24, 48},
                        new double[]{12, 36})) {
                    return Mode.Next;
                }
                break;
            case 1:
                if (robot.driveEngine.moveOnPath(
                        new double[]{36, 36},
                        new double[]{24, 24})) {
                    return Mode.Next;
                }
                break;
            case 2:
                if (robot.driveEngine.moveOnPath(
                        new double[]{48, 24},
                        new double[]{36, 12})) {
                    return Mode.Next;
                }
                break;
        }

        return Mode.Same;
    }


    private Mode slide2()
    {
        telemetry.addData("time", getTime());
        if(robot.driveEngine.moveOnPath(DriveEngine.Positioning.Absolute,
                new double[]{0, 56, Math.PI / 4}))
        {
            return Mode.Next;
        }
        return Mode.Same;
    }

    private void setiSP(double[] location)
    {
        double max = Math.max(Math.abs(location[0]), Math.abs(location[1]));
        iSP = max == Math.abs(location[1])? 1 : -1;
    }


//    If you are standing in the Red Alliance Station looking towards the center of the field,
//    The X axis runs from your left to the right. (positive from the center to the right)
//    The Y axis runs from the Red Alliance Station towards the other side of the field
//    where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
//    The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)

    private Mode waitToDetectPicture()
    {
        if(robot.name == Bogg.Name.Fakebot)
            return Mode.Next;

        double[] location = camera.getLocation();
        if(location != null)
        {
            setiSP(location);
            return Mode.Next;
        }
        return Mode.Same;
    }


    private Mode moveToDepot()
    {
        if(robot.driveEngine.moveOnPath(DriveEngine.Positioning.Absolute,
                new double[]{-iSP * 56, 56, iSP * Math.PI/2})){
            timer.reset();
            return Mode.Next;
        }
        return Mode.Same;
    }


    private Mode dropMarker()
    {
        robot.dropMarker(Bogg.Direction.Down);

        if(getTime() > .5) {  //time to drop marker
            return Mode.Next;
        }

        return Mode.Same;
    }


    private Mode moveToCrater()
    {
        robot.driveEngine.drive(0,2,true, 0,1);
        telemetry.addData("trueX", robot.driveEngine.trueX);
        telemetry.addData("usingImu", robot.sensors.usingImu);

        if(robot.sensors.usingImu)
            if (robot.sensors.isTilted())
                    return Mode.Next;

        if (robot.driveEngine.trueX * iSP > 24)
            return Mode.Next;


        return Mode.Same;
    }


    Mode stop()
    {
        robot.driveEngine.stop();
        telemetry.addLine("Done!!!");
        return Mode.Stop;
    }

    private double getTime()
    {
        return timer.seconds();
    }

    boolean rotateToWall(double accuracy_angle)
    {
        accuracy_angle = MyMath.radians(accuracy_angle);
        Double wallHeading = camera.headingToWall();
        telemetry.addData("wall heading", wallHeading);
        if(wallHeading != null)
        {
            if(Math.abs(wallHeading) < accuracy_angle) {
                return true;
            }
            else
            {
                if(wallHeading > 0)
                    robot.driveEngine.rotate(.01);
                else
                    robot.driveEngine.rotate(-.01);
            }

            return false;
        }
        return false;
    }

    boolean driveCurvyToWall()
    {
        double[] location = camera.getLocation();

        if(location != null)
        {
            VuforiaTrackable target = null;
            for(int i = 0; i < 4; i++)
                if(camera.targetVisible(i))
                    target = camera.allTrackables.get(i);

            double[] drive = camera.getMoveToWall(location, camera.getHeading(), target);
            double delta_x = drive[0];
            double delta_y = drive[1];
            double headingToTarget = drive[2];
            double r = Math.hypot(delta_x, delta_y);

            if(Math.abs(headingToTarget) < 5 && r < .5)
                return true;
            else
            {
                robot.driveEngine.moveOnPath("curvy",DriveEngine.Positioning.Relative,
                        drive);
            }
        }
        return false;
    }

    boolean driveCurvyToTarget(double[] driveTarget)
    {
        double[] location = camera.getLocation();

        if(location != null)
        {
            VuforiaTrackable target = null;
            for(int i = 0; i < 4; i++)
                if(camera.targetVisible(i))
                    target = camera.allTrackables.get(i);
            double[] wallTargetLocation = new double[2];

            wallTargetLocation[0] = Math.round(target.getLocation().getTranslation().get(0) / 25.4);
            wallTargetLocation[1] = Math.round(target.getLocation().getTranslation().get(1) / 25.4);

            double[] drive = camera.getMoveToPosition(location, camera.getHeading(), driveTarget);
            double delta_x = drive[0];
            double delta_y = drive[1];
            double headingToTarget = drive[2];
            double r = Math.hypot(delta_x, delta_y);

            if(Math.abs(MyMath.degrees(headingToTarget)) < 5 && r < .75)
                return true;
            else
            {
                robot.driveEngine.moveOnPath(DriveEngine.Positioning.Relative,
                        drive);
            }
        }
        return false;
    }

    void update()
    {
        robot.update();
    }
}
