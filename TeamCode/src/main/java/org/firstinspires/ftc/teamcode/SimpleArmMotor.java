package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SimpleArmMotor {
    final static int BUFFER_TICKS = 3;
    final static int totalTicks = 288;

    int maxTicks;
    int minTicks;
    DcMotor motor;

    public SimpleArmMotor(DcMotor motor, DcMotor.Direction direction, int minTicks, int maxTicks){
        this.minTicks = minTicks;
        this.maxTicks = maxTicks;

        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    boolean ticksInRange(int ticks) {
        return ticks > minTicks + BUFFER_TICKS && ticks < maxTicks - BUFFER_TICKS;
    }

    public void move(int ticks, double power) {
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower((float)power);
    }

    public void inc(int ticks, double power) {
        while(motor.isBusy());
        int current = motor.getCurrentPosition();
        if(ticksInRange(current + ticks))
            move(current + ticks, power);
        else
            move(maxTicks, power);
    }

    public void dec(int ticks, double power) {
        while(motor.isBusy());
        int current = motor.getCurrentPosition();
        if(ticksInRange(current - ticks))
            move(current - ticks, power);
        else
            move(minTicks, power);
    }

    public String getCurrentPosition() {
        return motor.getCurrentPosition() + " min: " + minTicks + " max: " + maxTicks;
    }

    public void manual(double power) {
        motor.setPower(power);
    }
}
