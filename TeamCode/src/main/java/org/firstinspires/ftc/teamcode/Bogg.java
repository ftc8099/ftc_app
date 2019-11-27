package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bogg
{
    Telemetry telemetry;
    SimpleDriveEngine driveEngine;

    public Bogg(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        driveEngine = new SimpleDriveEngine(hardwareMap);
    }

    void manualDrive(double x, double y, double spin)
    {
        if(spin > 0)
            driveEngine.rotate(spin);
        else
            driveEngine.drive(x, y);
    }

}
