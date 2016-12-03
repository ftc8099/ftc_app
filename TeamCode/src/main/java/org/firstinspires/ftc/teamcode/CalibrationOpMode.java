/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoOpmode", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class CalibrationOpMode extends LinearOpMode {

    //This is our timer
    private ElapsedTime timer = new ElapsedTime();

    //The engine which controls our drive motors
    DriveEngine engine = null;
    Sensors sensors = null;

    //Power constants
    private static final double LOW_POWER = .1;

    @Override
    public void runOpMode() {
        engine = new DriveEngine(DriveEngine.engineMode.directMode, hardwareMap, gamepad1);
        sensors = new Sensors(hardwareMap);
        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        timer.reset();

        //Run until the end of autonomous
        while (opModeIsActive())
        {
            //Run until we hit something
            while(sensors.rangeSensorFront.cmUltrasonic() && goingForward)
            {
                if(timer.seconds()<TIME_ONE)
                    engine.drive(LOW_POWER);
                else if(timer.seconds() < TIME_TWO)
                    engine.drive(MID_POWER);
                else
                    engine.drive(HIGH_POWER);
            }

            //Reset the timer
            timer.reset();

            engine.stop();

            telemetry.addLine("Stopped");
            telemetry.update();
            idle();     // allow something else to run (aka, release the CPU)
        }
    }
}
