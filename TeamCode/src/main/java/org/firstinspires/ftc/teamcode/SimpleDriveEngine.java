package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimpleDriveEngine {
    DcMotor right;
    DcMotor left;
    DcMotor back;

    double offsetAngle;

    public SimpleDriveEngine(HardwareMap hardwareMap) {

        right = hardwareMap.dcMotor.get("Right");
        left  = hardwareMap.dcMotor.get("Left");
        back  = hardwareMap.dcMotor.get("Back");
        offsetAngle = 0;

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(DcMotor.Direction.FORWARD);
        left.setDirection (DcMotor.Direction.FORWARD);
        back.setDirection (DcMotor.Direction.FORWARD);
    }

    public void drive(double x, double y) {
        //Calculates the an offset angle if we need one
        double xprime = x * Math.cos(offsetAngle) - y * Math.sin(offsetAngle);
        double yprime = x * Math.sin(offsetAngle) + y * Math.cos(offsetAngle);

        x = xprime;
        y = yprime;

        right.setPower( (-x / 2) + ( ( y * Math.sqrt(3) ) / 2 ) );
        left.setPower ( (-x / 2) + ( (-y * Math.sqrt(3) ) / 2 ) );
        back.setPower ( x );
    }

    public void rotate(double y) {
        back.setPower (y);
        right.setPower(y);
        left.setPower (y);

    }
}
