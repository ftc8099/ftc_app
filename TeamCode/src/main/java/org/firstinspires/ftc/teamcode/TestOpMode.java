package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TestOpMode", group="")
public class TestOpMode extends LinearOpMode {

    ArmMotor testMotor = null;
    int ticks = 0;

    @Override
    public void runOpMode() {

        testMotor = new ArmMotor(hardwareMap.get(DcMotor.class, "elbow2"),DcMotor.Direction.REVERSE,288,-216,216,13.164);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.y){
                try{
                    testMotor.moveTo(ticks+36);
                    telemetry.addData("Status", "successfully moved motor");
                    ticks+=36;
                }catch (Exception e){
                    telemetry.addData("Status", e);
                }
            }else if(gamepad1.a){
                try{
                    testMotor.moveTo(ticks-36);
                    telemetry.addData("Status", "successfully moved motor");
                    ticks-=36;
                }catch (Exception e){
                    telemetry.addData("Status", e);
                }
            }

            telemetry.update();

            //telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}