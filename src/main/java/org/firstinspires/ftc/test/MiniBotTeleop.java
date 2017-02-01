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
package org.firstinspires.ftc.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.berean.robotics.robots.test.HardwareMiniBot;

/**
 * This file provides  Telop driving for Minibot.
 */

@TeleOp(name="Minibot: Teleop", group="mini")
//@Disabled
public class MiniBotTeleop extends OpMode{

    /* Declare OpMode members. */

    HardwareMiniBot robot = new HardwareMiniBot(); // use the class created to define a Aimbot's hardware
    boolean sniperModeOn = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        if (!sniperModeOn) {
            robot.frontLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
        }
        else
        {
            robot.frontLeftMotor.setPower(left/2);
            robot.frontRightMotor.setPower(right/2);
        }

        if (gamepad1.x)
        {
            sniperModeOn = true;
        }
        if (gamepad1.y)
        {
            sniperModeOn = false;
        }
/**
        if (gamepad1.left_trigger > 0)
            robot.pusherLeft.setPosition(0.5);
        else
            robot.pusherLeft.setPosition(0);

        if (gamepad1.right_trigger > 0)
            robot.pusherRight.setPosition(0.6);
        else
            robot.pusherRight.setPosition(0);


**/
        // this logic assumes the color sensor is on the right
        int color = robot.colorSensor.getColorNumber();

        if (color == robot.colorSensor.SENSOR_RED) {
            robot.redLED(true);
            robot.blueLED(false);
        } else if (color == robot.colorSensor.SENSOR_BLUE) {
            robot.redLED(false);
            robot.blueLED(true);
          } else {
            robot.redLED(false);
            robot.blueLED(false);

        }


        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("light", "%f", robot.lightSensor.getLightDetected());
        telemetry.addData("ColorNumber: %d", color);

        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
