package edu.berean.robotics.robots.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import edu.berean.robotics.dopple.core.DoppleBot;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a AimBot.
 *
 */
public class HardwareMiniDoppleBot extends DoppleBot
{
    private static String LOG_TAG = "Hardware MINI DOPPLEBOT - ";
    private static String FRONT_LEFT_MOTOR_NAME = "left_front";
    private static String FRONT_RIGHT_MOTOR_NAME = "right_front";
    private static String RIGHT_BUTTON_PUSHER = "pusher2";
    private static String LEFT_BUTTON_PUSHER = "servo";
    private static String LIGHT_SENSOR = "light";
    private static int MAX_SPEED_FOR_ANDYMARK = 2184; //actual max is 2800  making max 80% to handle battery power loss


    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public LightSensor lightSensor = null;
    public Servo pusherLeft = null;
    public Servo pusherRight = null;
    public DigitalChannel r;
    public DigitalChannel g;
    public DigitalChannel b;
    public boolean encoderDriveIsEnabled = false;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();



    /* Constructor */
    public HardwareMiniDoppleBot(){

    }

    /* handle standard motor initialization */
    private DcMotor initMotor(String name, boolean reverse) {
        RobotLog.i(LOG_TAG + " initializing motor: " + name);
        DcMotor motor = hwMap.dcMotor.get(name);

        if(!encoderDriveIsEnabled) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMaxSpeed(MAX_SPEED_FOR_ANDYMARK);
        }

        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);

        return motor;
    }


    /* handle standard servo initialization */
    private Servo initServo(String name, double pos, boolean reverse) {
        RobotLog.i(LOG_TAG + " initializing servo " + name);
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        srv.setPosition(pos);
        return srv;
    }

    public void startRobot(){

    }

    public void stopRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void initializeRobot(HardwareMap ahwMap) {
        // Save reference to Hardware map
        RobotLog.i(LOG_TAG + " hardware - initializeRobot");
        RobotLog.i(LOG_TAG + "ENCODER drive:" + encoderDriveIsEnabled);
        hwMap = ahwMap; // initialize before calling other init functions

        // Define and Initialize Motors

        frontLeftMotor  = initMotor(FRONT_LEFT_MOTOR_NAME, true);
        frontRightMotor = initMotor(FRONT_RIGHT_MOTOR_NAME, false);
        pusherLeft = initServo(LEFT_BUTTON_PUSHER, 0.1, false);
        //pusherRight = initServo(RIGHT_BUTTON_PUSHER, 0.1, true);
        lightSensor = hwMap.lightSensor.get(LIGHT_SENSOR);
        lightSensor.enableLed(false);

        addRobotComponent(FRONT_LEFT_MOTOR_NAME, frontLeftMotor);
        addRobotComponent(FRONT_RIGHT_MOTOR_NAME, frontRightMotor);
        addRobotComponent(LEFT_BUTTON_PUSHER, pusherLeft);
        //robotComponents.put(RIGHT_BUTTON_PUSHER, pusherLeft);
        //robotComponents.put(LEFT_BUTTON_PUSHER, pusherLeft);

        /*r = hwMap.digitalChannel.get("r");
        r.setMode(DigitalChannelController.Mode.OUTPUT);
        g = hwMap.digitalChannel.get("g");
        g.setMode(DigitalChannelController.Mode.OUTPUT);
        b = hwMap.digitalChannel.get("b");
        b.setMode(DigitalChannelController.Mode.OUTPUT);*/

    }


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

