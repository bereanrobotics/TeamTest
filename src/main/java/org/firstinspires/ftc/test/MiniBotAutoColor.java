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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.berean.robotics.robots.test.HardwareMiniBot;

@Autonomous(name="MiniBot: AutoColor", group="mini")
//@Disabled

public class MiniBotAutoColor extends LinearOpMode{

    HardwareMiniBot robot = new HardwareMiniBot(); // use the class created to define a Aimbot's hardware
    private ElapsedTime runtime = new ElapsedTime();  // required for delay


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     WHITE_THRESHOLD = 0.1;  // spans between 0.1 - 0.5 from dark to light
    static final double     DRIVE_SPEED             = 0.15;
    static final double     SLOW_DRIVE_SPEED             = 0.1;

    private double lightmax = 0.0;

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.setTargetPosition(leftInches, rightInches);
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.drive(speed, speed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // reset the timeout time and start motion.
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.isDriving()) {
                // Allow time for other processes to run.
                idle();
            }
        }

        // Stop all motion;
        robot.drive(0, 0);
        // Turn off RUN_TO_POSITION
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private boolean driveToLine(double speed, double timeout)throws InterruptedException {
        // run until the white line is seen OR the driver presses STOP;
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive(speed, speed);
        runtime.reset();
        double light = robot.lightSensor.getLightDetected();

        while (opModeIsActive() && (runtime.seconds() < timeout) && (light < WHITE_THRESHOLD)) {

            if (light > lightmax) lightmax = light;
            // Display the light level while we are looking for the line
            telemetry.addData("Drive Level", robot.lightSensor.getLightDetected());
            telemetry.update();
            light = robot.lightSensor.getLightDetected();
            idle();
        }
        //slowly back up to find edge of line
        robot.drive(-0.05,-0.05);
        while (opModeIsActive() && (light > lightmax/2)) {

            if (light > lightmax) lightmax = light;
            // Display the light level while we are looking for the line
            telemetry.addData("Finding Edge", robot.lightSensor.getLightDetected());
            telemetry.update();
            light = robot.lightSensor.getLightDetected();
            idle();
        }

        robot.drive(0, 0);
        return (light < WHITE_THRESHOLD);
    }

    private void followLine(double speed, double timeout, boolean rightHand)throws InterruptedException {

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timeout) ) {
            double light = robot.lightSensor.getLightDetected();
            if (light > lightmax) lightmax = light;
            double level = light / lightmax;
            double leftSpeed;
            double rightSpeed;
            if (rightHand) {
                leftSpeed = speed*level;
                rightSpeed = speed-(speed*level);
            } else {
                leftSpeed = speed-(speed*level);
                rightSpeed = speed*level;
            }
            robot.drive(leftSpeed, rightSpeed);

            double distance = robot.distance.getUltrasonicLevel();


            // Display the light level while we are looking for the line
            telemetry.addData("Follow Level", light);
            telemetry.addData("LeftSpeed", leftSpeed);
            telemetry.addData("RightSpeed", rightSpeed);
            telemetry.addData("Distance", distance);
            telemetry.update();

            if (distance > 2 && distance < 10) break;

            //if (robot.colorSensor.getColorNumber() > 0) break;
            idle();
        }
        robot.drive(0, 0);
        return;
    }

    private void watchColor(double timeout)throws InterruptedException {

        encoderDrive(.15, 2, 2, 10); // move forward 2 inches
        encoderDrive(.15, -2, -2, 10); // move forward 2 inches
        //encoderDrive(.15, 2, 2, 10); // move forward 2 inches
        runtime.reset();


        while (opModeIsActive() && (runtime.seconds() < timeout)) {
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
            telemetry.addData("Watch light", "%f", robot.lightSensor.getLightDetected());
            telemetry.addData("ColorNumber: ", "%d", color);
            telemetry.addData("Distance", "%f", robot.distance.getUltrasonicLevel());
            updateTelemetry(telemetry);
            idle();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (opModeIsActive()) {
            driveToLine(DRIVE_SPEED, 50000);
            followLine(SLOW_DRIVE_SPEED,50000,true);
            watchColor(10000);
        }
    }

}
