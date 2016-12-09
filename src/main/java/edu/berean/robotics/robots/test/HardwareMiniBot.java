package edu.berean.robotics.robots.test;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a AimBot.
 *
 */
public class HardwareMiniBot
{

    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public ColorSensor colorSensor = null;
    public LightSensor lightSensor = null;
    public Servo pusherLeft = null;
    public Servo pusherRight = null;
    public DigitalChannel r;
    public DigitalChannel g;
    public DigitalChannel b;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMiniBot(){

    }

    /* handle standard motor initialization */
    private DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }


    /* handle standard servo initialization */
    private Servo initServo(String name, double pos, boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        srv.setPosition(pos);
        return srv;
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap; // initialize before calling other init functions

        // Define and Initialize Motors

        frontLeftMotor  = initMotor("left_front", false);
        frontRightMotor = initMotor("right_front", true);
        //pusherLeft = initServo("pusher1", 0.1, false);
        //pusherRight = initServo("pusher2", 0.1, true);
        lightSensor = hwMap.lightSensor.get("light");
        //colorSensor = hwMap.colorSensor.get("color");
        //colorSensor.enableLed(false);
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

