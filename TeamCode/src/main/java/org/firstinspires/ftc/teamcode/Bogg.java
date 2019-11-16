package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bogg
{
    Telemetry telemetry;
    SimpleDriveEngine driveEngine;
    SimpleArm arm;

    public Bogg(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        driveEngine = new SimpleDriveEngine(hardwareMap);
        arm = new SimpleArm(hardwareMap);
    }

    void manualDrive(double x, double y, double spin)
    {
        if(spin != 0)
            driveEngine.rotate(spin);
        else
            driveEngine.drive(x, y);

        telemetry.addData("Drive Right: ", driveEngine.right.getPower());
        telemetry.addData("Drive  Left: ",  driveEngine.left.getPower());
        telemetry.addData("Drive  Back: ",  driveEngine.back.getPower());
    }

    void moveArm(boolean elbow1Down, boolean elbow1Up, boolean elbow2Down, boolean elbow2Up) {
//        if(elbow1Down)
//            arm.decElbow1();
//        else if(elbow1Up)
//            arm.incElbow1();
//        else if(elbow2Down)
//            arm.decElbow2();
//        else if(elbow2Up)
//            arm.incElbow2();
//
//
//        telemetry.addData("Arm elbow1: ", arm.elbow1.getCurrentPosition());
//        telemetry.addData("Arm elbow2: ", arm.elbow2.getCurrentPosition());

        if(elbow1Down) {
            arm.moveManually("elbow1", -1);
        }else if(elbow1Up){
            arm.moveManually("elbow1",1);
        }else{
            arm.moveManually("elbow1",0);
        }
        if(elbow2Down) {
            arm.moveManually("elbow2", -1);
        }else if(elbow2Up){
            arm.moveManually("elbow2",1);
        }else{
            arm.moveManually("elbow2",0);
        }
    }

    void getBlock(boolean open,boolean close) {
        if(open)
            arm.openWrist();
        if(close)
            arm.closeWrist();

        telemetry.addData("Arm Wrist: ", arm.wrist.getPosition());
    }
}
