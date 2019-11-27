package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class ArmMotor{

    final static int BUFFER_TICKS = 3;

    DcMotor motor;
    int ticks;

    int maxTicks;
    int minTicks;
    public double lengthInches;

    public ArmMotor(DcMotor motor, DcMotor.Direction direction, int ticks, int minTicks, int maxTicks, double lengthInches){
        this.motor = motor;
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.ticks = ticks;
        this.maxTicks = maxTicks;
        this.minTicks = minTicks;
        this.lengthInches = lengthInches;
    }

    public void moveTo(int ticks) throws Exception {
        if(ticks>minTicks+BUFFER_TICKS && ticks<maxTicks-BUFFER_TICKS && !motor.isBusy()){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);
            motor.setPower(1.0);
        }else{
            if(motor.isBusy()){
                throw new Exception("Motor function interruption attempted\nTarget: "+ticks+"\nActual Position: "+motor.getCurrentPosition());
            }else {
                throw new Exception("Target outside of allowed range\nTarget: "+ticks+"\nActual Position: "+motor.getCurrentPosition());
            }
        }
    }

    public void moveToDegrees(int angle) throws Exception {
        moveTo(angle*ticks/360);
    }

    public void moveToRadians(double angle) throws Exception {
        moveTo((int)Math.round(angle*ticks/(2*Math.PI)));
    }

    public void calibrate(DigitalChannel limitSwitch){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(limitSwitch.getState() == true){
            motor.setPower(0.15);
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}