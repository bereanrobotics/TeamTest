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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.berean.robotics.robots.test.HardwareMiniBot;

/**
 * This file provides  Telop driving for Aimbot.
 */

@Autonomous(name="MiniBot: AutoColor", group="mini")
@Disabled

public class MiniBotAutoColor extends LinearOpMode{

    /* Declare OpMode members. */

    HardwareMiniBot robot = new HardwareMiniBot(); // use the class created to define a Aimbot's hardware


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1:  Drive forward for 1 seconds
        /*robot.frontLeftMotor.setPower(FORWARD_SPEED);
        robot.backLeftMotor.setPower(FORWARD_SPEED);
        robot.frontRightMotor.setPower(FORWARD_SPEED);
        sleep(1000);
        // Step 2:  Spin right for 1.3 seconds
        robot.frontLeftMotor.setPower(TURN_SPEED);
        robot.backLeftMotor.setPower(TURN_SPEED);
        robot.frontRightMotor.setPower(-TURN_SPEED);
        robot.backRightMotor.setPower(-TURN_SPEED);
        sleep(1000);
        // Step 3:  Drive Backwards for 1 Second
        robot.frontLeftMotor.setPower(-FORWARD_SPEED);
        robot.backLeftMotor.setPower(-FORWARD_SPEED);
        robot.frontRightMotor.setPower(-FORWARD_SPEED);
        robot.backRightMotor.setPower(-FORWARD_SPEED);
        sleep(1000);
        // Step 4:  Stop and close the claw.
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);*/
        while(true) {

            // convert the RGB values to HSV values.
            Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);


            if (hsvValues[0] <  50) {
                robot.pusherLeft.setPosition(0.5);
                /*telemetry.addData("redFound", "%d", robot.colorSensor.red());
                telemetry.addData("pusherLeft", "%.2f", 0.5);
                updateTelemetry(telemetry);
                */
                idle();
                sleep(3000);
                robot.pusherLeft.setPosition(0.1);
                idle();
                sleep(1000);

            } else if (hsvValues[0] > 200) {
                robot.pusherRight.setPosition(0.5);
                /*telemetry.addData("blueFound", "%d", robot.colorSensor.blue());
                telemetry.addData("pusherRight", "%.2f", 0.5);
                updateTelemetry(telemetry);
                */

                robot.pusherRight.setPosition(0.1);
                idle();
                sleep(1000);
            }
            //telemetry.addData("red", "%d", robot.colorSensor.red());
            //telemetry.addData("blue", "%d", robot.colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            updateTelemetry(telemetry);
            idle();
            sleep(500);
        }


        /*robot.frontLeftMotor.setPower(FORWARD_SPEED);
        robot.frontRightMotor.setPower(FORWARD_SPEED);
        sleep(500);
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);*/
    }

}
