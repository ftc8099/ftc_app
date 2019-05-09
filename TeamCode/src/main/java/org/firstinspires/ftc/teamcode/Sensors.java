package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

class Sensors {
    HardwareMap hardwareMap;

    private TouchSensor touchTop;
    private TouchSensor touchBottom;

    BNO055IMU imu = null;

    boolean usingImu = false;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    SensorEventListener sensorEventListener;

    Bogg.Name name;


    Sensors(HardwareMap hardwareMap, Bogg.Name whichRobot)
    {
        this.hardwareMap = hardwareMap;

        this.name = whichRobot;

        sensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {
                switch (sensorEvent.sensor.getType()) {
                    case Sensor.TYPE_ACCELEROMETER: {
                        System.arraycopy(sensorEvent.values, 0, gravity1, 0, gravity1.length);
                        break;
                    }
                    case Sensor.TYPE_MAGNETIC_FIELD: {
                        System.arraycopy(sensorEvent.values, 0, geomagnetic, 0, geomagnetic.length);
                        break;
                    }
                }
                updateOrientationAngles();
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };

        switch (name)
        {
            case Bogg:
                touchTop = hardwareMap.get(TouchSensor.class, "touchTop");
                touchBottom = hardwareMap.get(TouchSensor.class, "touchBottom");
                //dLow = hardwareMap.get(DistanceSensor.class, "dLow");
                //dHigh = hardwareMap.get(DistanceSensor.class, "dHigh");
            case MiniBogg:
                usingImu = true;
                if(imu == null) {
                    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                    parameters.loggingEnabled = true;
                    parameters.loggingTag = "IMU";
                    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                    // and named "imu".
                    try {
                        imu = hardwareMap.get(BNO055IMU.class, "imu");
                        imu.initialize(parameters);
                    } catch (IllegalArgumentException i) {
                        usingImu = false;
                    }
                }
            case Fauxbot:
            case Fakebot:
        }
    }

    /**
     *
     * @return angle in radians
     */
    double getImuHeading()
    {
        if(usingImu)
            return MyMath.radians(imu.getAngularOrientation().firstAngle);
        return 0;
    }

    boolean touchTopIsPressed() {
        return name != Bogg.Name.Bogg || touchTop.isPressed();
    }

    boolean touchBottomIsPressed() {
        return touchBottom.isPressed();
    }


    /**
     * This method could be used as a template for a distance sensor
     * I would recommend a sliding average filter, but you could also use exponential.
     */
    double highAverage = 0;
    double highAlpha = .4;
    ArrayList<Double> highDistances = new ArrayList<>();
    double getHighDistance()
    {
        double d = 30;
        highDistances.add(d);
        if(highDistances.size() > 5)
            highDistances.remove(0);

        highAverage = MyMath.median(highDistances) * highAlpha + highAverage * (1 - highAlpha);
        return highAverage;
    }

    MyColorSensor myColorSensor;
    void initializeColorSensor()
    {
        if(myColorSensor == null)
            myColorSensor = new MyColorSensor(hardwareMap);
    }

    boolean isGold()
    {
        return myColorSensor.isColor(MyColorSensor.Color.Gold);
    }

    boolean isTilted()
    {
        if(usingImu)
        {
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return Math.hypot(orientation.secondAngle, orientation.thirdAngle) > 10;
        }
        return false;
    }

    float[] gravity1 = new float[3];
    float[] geomagnetic = new float[3];
    float[] R = new float[16];
    float[] orientation = new float[3];

    float azimuth;
    float pitch;
    float roll;

    private void updateOrientationAngles() {
        SensorManager.getRotationMatrix(R, null, gravity1, geomagnetic);
        SensorManager.getOrientation(R, orientation);
        azimuth = orientation[0];
        pitch = orientation[1];
        roll = orientation[2];
    }

    void update()
    {
        updateOrientationAngles();
    }

}
