package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SimpleArm {
    int ticks = 10; //The amount of ticks to be armMotor will inc or dec
    double power1 = 1; //Power arm motors will move at
    double power2 = 0; //Power arm motors will move at

    double openPosition = 1; //Positions for open wrist servo
    double closePosition = 0; //Position for close wrist servo

    SimpleArmMotor elbow1;
    SimpleArmMotor elbow2;
    Servo wrist;

    public SimpleArm(HardwareMap hardwareMap) {
        elbow1 = new SimpleArmMotor(
                hardwareMap.dcMotor.get("elbow1"),
                DcMotor.Direction.FORWARD,
                0, //Start Position
                133); //270 degrees

        elbow2 = new SimpleArmMotor(
                hardwareMap.dcMotor.get("elbow2"),
                DcMotor.Direction.FORWARD,
                0, //Start Position
                120); //270 degrees

        wrist = hardwareMap.servo.get("wrist");
    }

    public void incElbow1() {
        elbow1.inc(ticks, power1);
    }

    public void decElbow1() {
        elbow1.dec(ticks, power1);
    }

    public void incElbow2() {
        elbow2.inc(ticks, power1);
    }

    public void decElbow2() {
        elbow2.dec(ticks, power1);
    }

    public void moveManually(String name, int dir){ // dir needs to be 1 , 0 or -1
        switch(name){
            case "elbow1":
                elbow1.manual(power1*dir);
                break;
            case "elbow2":
                elbow2.manual(power2*dir);
                break;
            default:
                break;
        }
    }

    public void openWrist() {
        wrist.setPosition(openPosition);
    }

    public void closeWrist() {
        wrist.setPosition(closePosition);
    }
}
