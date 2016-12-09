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
import com.qualcomm.robotcore.util.RobotLog;

import edu.berean.robotics.robots.test.HardwareMiniDoppleBot;

/**
 * This file provides  Telop driving for Minibot.
 */

@TeleOp(name="Minibot: RecordTeleop", group="MINI DOPPLE")
//@Disabled
public class MiniDoppleBotTeleop extends OpMode{

    private String LOG_TAG = "MiniDoppleBotTeleop - ";

    /* Declare OpMode members. */

    protected HardwareMiniDoppleBot robot = new HardwareMiniDoppleBot(); // use the class created to define a Aimbot's hardware
    protected boolean sniperModeOn = true;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        RobotLog.i(LOG_TAG + "initializing");
        robot.initializeRobot(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initializing. Encoder drive is " + robot.encoderDriveIsEnabled);
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
        RobotLog.i(LOG_TAG + "starting");
        telemetry.addData("Status", "starting");
        telemetry.update();

        robot.startRobot();
        robot.startRecording();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double pusherLeftInput;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        pusherLeftInput = gamepad1.left_trigger;
        if (!sniperModeOn) {
            robot.frontLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
        }
        else
        {
            robot.frontLeftMotor.setPower(left/3);
            robot.frontRightMotor.setPower(right/3);
        }

        if (gamepad1.x)
        {
            sniperModeOn = true;
        }
        if (gamepad1.y)
        {
            sniperModeOn = false;
        }

        robot.pusherLeft.setPosition(pusherLeftInput);

        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "left power %.2f  position %d", robot.frontLeftMotor.getPower(), robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("right", "right power %.2f position %d", robot.frontRightMotor.getPower(), robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("light", "%f", robot.lightSensor.getLightDetected());
        telemetry.addData("servo", "%.2f", robot.pusherLeft.getPosition());
        telemetry.update();

        if(robot.robotRecordingIsOn()) {
            RobotLog.i(LOG_TAG + "update recording");
            robot.updateRecording();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        RobotLog.i(LOG_TAG + "stopping");
        telemetry.addData("Status", "stopping");
        telemetry.update();
        robot.stopRecording();
        robot.stopRobot();
    }

}
