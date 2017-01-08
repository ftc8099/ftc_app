package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {
    HardwareMap hardwareMap = null;

    //The touch sensor devices
    DeviceInterfaceModule dim = null;
    TouchSensor touchSensor = null;

    //Probably wont use these...
    OpticalDistanceSensor opSensorBottom = null;
    OpticalDistanceSensor opSensor2 = null;

    ColorSensor colorSensorLeft = null;
    ColorSensor colorSensorRight = null;
    ColorSensor colorSensorBottom = null;

    //Distance between the range sensors
    private double distanceApart;
    //Direction
    private boolean forwards;

    ModernRoboticsI2cRangeSensor rangeSensorFront = null;
    ModernRoboticsI2cRangeSensor rangeSensorBack = null;

    final static private int COLOR_MAX = 255;
    final static private int RED_COLOR = 255;
    final static private int BLUE_COLOR = 255;
    final static private int WHITE_COLOR = 240;

    public Sensors(HardwareMap hardwareMap) {
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        opSensorBottom = hardwareMap.opticalDistanceSensor.get("op_sense1");
//        opSensor2 = hardwareMap.opticalDistanceSensor.get("op_sense2");
//        touchSensor = hardwareMap.touchSensor.get("sensor_touch");
 //       colorSensorLeft = hardwareMap.colorSensor.get("sensor_color_left");
//        colorSensorRight = hardwareMap.colorSensor.get("sensor_color_right");
//        colorSensorBottom = hardwareMap.colorSensor.get("sensor_color_bottom");
//        rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "");
//        rangeSensorBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "");
//        colorSensorBottom = hardwareMap.
    }
//
//    public OpticalDistanceSensor getOp(int index){
//        touchSensor = hardwareMap.touchSensor.get("sensor_touch");
//
//        opSensor1 = hardwareMap.opticalDistanceSensor.get("sensor_optical1");
//        opSensor2 = hardwareMap.opticalDistanceSensor.get("sensor_optical2");
//
//        colorSensorLeft = hardwareMap.colorSensor.get("sensor_color_left");
//        colorSensorRight = hardwareMap.colorSensor.get("sensor_color_right");
//        colorSensorBottom = hardwareMap.colorSensor.get("sensor_color_bottom");
//
//        rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front");
//        rangeSensorBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_back");
//    }

    boolean isLeftRed() {
        if (colorSensorLeft.red() >= RED_COLOR)
            return true;
        else
            return false;
    }

    boolean isRightRed() {
        if (colorSensorRight.red() >= RED_COLOR)
            return true;
        else
            return false;

    }

    boolean isLeftBlue() {
        if (colorSensorLeft.blue() >= BLUE_COLOR)
            return true;
        else
            return false;
    }

    boolean isRightBlue() {
        if (colorSensorRight.blue() >= BLUE_COLOR)
            return true;
        else
            return false;
    }

    boolean isBottomWhite()
    {
        if(colorSensorBottom != null && colorSensorBottom.red() >= COLOR_MAX & colorSensorBottom.blue() >= COLOR_MAX & colorSensorBottom.green() >= COLOR_MAX)
            return true;

        else if (colorSensorBottom.alpha() >= WHITE_COLOR) {
           return true;
        } else {
            return false;
        }
    }

    boolean opIsBottomWhite()
    {
        return opSensorBottom.getLightDetected() > 200;
    }

    double getFrontDistance()
    {
        return rangeSensorFront.getDistance(DistanceUnit.INCH);
    }

    double getBackDistance()
    {
        return rangeSensorBack.getDistance(DistanceUnit.INCH);
    }

    public OpticalDistanceSensor getOp(int index) {
        if (index == 1)
            return opSensorBottom;
        if (index == 2)
            return opSensor2;
        return null;
    }

    public double meanDistance()
    {
        return (rangeSensorBack.getDistance(DistanceUnit.INCH)+rangeSensorFront.getDistance(DistanceUnit.INCH))/2;
    }

    public double minDistanceAway()
    {
        return Math.min(rangeSensorBack.getDistance(DistanceUnit.INCH), rangeSensorFront.getDistance(DistanceUnit.INCH));
    }

    public double angleToWall()
    {
        double frontDistance = rangeSensorFront.getDistance(DistanceUnit.INCH);
        double backDistance = rangeSensorBack.getDistance(DistanceUnit.INCH);
        //gives positive values when angled towards wall
        double difference = backDistance - frontDistance;

        if(forwards){
            return Math.atan(difference/distanceApart);
        }
        else{
            return -1 * Math.atan(difference/distanceApart);
        }

    }

    public void invertDirection()
    {
        forwards = !forwards;
    }
}
