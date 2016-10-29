/* Copyright (c) 2016 PSM

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/*
 * This is an example LinearOpMode that shows how to use the touch sensor on the
 * the Modern Robotics Device Interface Module.  In addition, it shows how to use the Red and Blue LED
 *
 * This op mode assumes that there is a Device Interface Module attached, named 'dim'.
 * On this DIM there is a digital input named 'touch0'
 *
 * The Red/Blue LED will be used to indicate the state of the touch sensor.
 * Blue = false (0V), Red = true (5V)
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/
@TeleOp(name = "TouchTest", group = "Testing")
//@Disabled
public class TouchTest extends LinearOpMode {

  final int BLUE_LED_CHANNEL = 0;      // address for BLUE LED on the Device Interface Module
  final int RED_LED_CHANNEL = 1;       // address for RED LED on the Device Interface Module

  @Override
  public void runOpMode() {

    boolean               inputPin;             // Input State
    boolean               outputPin;            // Output State
    DeviceInterfaceModule dim;                  // Device Object
    DigitalChannel        digIn;                // Device Object
    DigitalChannel        digOut;               // Device Object

    // get a reference to a Modern Robotics DIM, and IO channels.
    dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping
    digIn = hardwareMap.get(DigitalChannel.class, "touch");    //  Use generic form of device mapping

    digIn.setMode(DigitalChannelController.Mode.INPUT);          // Set the direction of each channel

    // wait for the start button to be pressed.
    telemetry.addData("", "Test program for touch sensor");
    telemetry.addData("", "Uses Device Interface Modeule <dim> with <touch> sensor");
    telemetry.update();
    waitForStart();

    while (opModeIsActive())  {

        inputPin = digIn.getState();    //  Read the input pin

        // Display input pin state on LEDs
        if (inputPin) {
            dim.setLED(RED_LED_CHANNEL, true);
            dim.setLED(BLUE_LED_CHANNEL, false);
        }
        else {
            dim.setLED(RED_LED_CHANNEL, false);
            dim.setLED(BLUE_LED_CHANNEL, true);
        }

        telemetry.addData("Input", inputPin );
        telemetry.addData("LED",   inputPin ? "Red" : "Blue" );
        telemetry.update();

        idle();     // allow something else to run (aka, release the CPU)
    }
  }
}