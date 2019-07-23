package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.util.MyMath;

public class Screen {

    private static final double screenHeight = 1000;
    private static final double screenLength = 700;

    private static final double fieldInches = 144;

    private static double xScalar = 4.5;
    private static double yScalar = 5.5;

    Runnable runnable;

    private boolean wasTouched;
    private double[] fieldCoordinates = new double[2];

    Screen(){
        runnable = new Runnable() {
            @Override
            public void run() {
                onTouchEvent();
            }
        };
        FtcRobotControllerActivity.a.r = runnable;
    }


    private static double getScreenX(double fieldX)
    {
        return screenLength / 2  +  fieldX * xScalar;
    }
    private static double getFieldX(double screenX)
    {
        return (screenX - screenLength / 2) / xScalar;
    }
    private static double getScreenY(double fieldY)
    {
        return screenHeight / 2 - yScalar * fieldY;
    }
    private static double getFieldY(double screenY)
    {
        return (screenY - screenHeight / 2) * screenY;
    }

    static void moveRobotToPosition(double fieldX, double fieldY, double spin)
    {
        FtcRobotControllerActivity.moveRobot(getScreenX(fieldX), getScreenY(fieldY), MyMath.degrees(spin));
    }

    private void onTouchEvent()
    {
        wasTouched = true;
        fieldCoordinates[0] = getFieldX(FtcRobotControllerActivity.imageX);
        fieldCoordinates[1] = getFieldY(FtcRobotControllerActivity.imageY);
    }

    boolean wasTouched() {
        if(wasTouched) {
            wasTouched = false;
            return true;
        }

        return false;
    }

    double[] getFieldCoordinates()
    {
        return fieldCoordinates;
    }
}


