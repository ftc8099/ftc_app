package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class Arm {

    ArmMotor shoulder;
    ArmMotor elbow1;
    ArmMotor elbow2;
    DcMotor wrist;

    public Arm(ArmMotor shoulder, ArmMotor elbow1, ArmMotor elbow2, DcMotor wrist) {
        this.shoulder = shoulder;
        this.elbow1 = elbow1;
        this.elbow2 = elbow2;
        this.wrist = wrist;
    }

    public void targetXYInches(int xInches, int yInches) throws Exception{
        double dist = Math.sqrt((xInches * xInches) + (yInches * yInches));
        double theta = (xInches <= 0) ? Math.atan(yInches / xInches) : Math.atan(yInches / xInches) + Math.PI;
        if (dist <= shoulder.lengthInches + elbow1.lengthInches + elbow2.lengthInches) {
            double alpha = angleFunction(shoulder.lengthInches, elbow1.lengthInches, elbow2.lengthInches, dist);
            double beta = angleFunction(dist, shoulder.lengthInches, elbow1.lengthInches, elbow2.lengthInches);
            shoulder.moveToRadians(alpha + theta);
            elbow1.moveToRadians(beta);
            elbow2.moveToRadians(alpha);
        } else {
            throw new Exception("Target out of reach");
        }
    }

    private double angleFunction(double a, double b, double c, double d) {
        return Math.acos(((a * a) + (d * d) - (b * b) - (c * c)) / (2 * ((a * d) - (b * c))));
    }
}
