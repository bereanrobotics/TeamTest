package edu.berean.robotics.robots.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import edu.berean.robotics.BereanHardware;
import edu.berean.robotics.ColorNumberSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a AimBot.
 *
 */
public class HardwareMiniBot
{
    static final double     COUNTS_PER_MOTOR_REV    = 28.0; // 1120 or 28? eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    /* Public OpMode members. */
    public DeviceInterfaceModule cdi = null; // core device interface
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public OpticalDistanceSensor lightSensor;
    public ColorNumberSensor colorSensor;
    public UltrasonicSensor distance;


    /*
    public DigitalChannel r;
    public DigitalChannel g;
    public DigitalChannel b;
*/
    /* Constructor */
    public HardwareMiniBot(){

    }
    protected HardwareMap hwMap           =  null;
    protected ElapsedTime period  = new ElapsedTime();


    /* handle standard motor initialization */
    protected DcMotor initMotor(String name, boolean reverse) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    protected DcMotor initMotorWithEncoder(String name, boolean reverse, boolean zeroBreak) {
        DcMotor motor = hwMap.dcMotor.get(name);
        if (reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (zeroBreak) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        return motor;
    }


    /* handle standard servo initialization */
    protected Servo initServo(String name, double pos, boolean reverse) {
        Servo srv = hwMap.servo.get(name);
        if (reverse) srv.setDirection(Servo.Direction.REVERSE);
        //srv.setPosition(pos); do not do this here since it will cause motion when we don't want it.
        return srv;
    }

    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor  = initMotor("left_front", false);
        frontRightMotor = initMotor("right_front", true);

        // get a reference to our Light Sensor object.
        lightSensor = ahwMap.opticalDistanceSensor.get("light");

        // save a reference to the core device interface to set LED lights
        cdi = ahwMap.deviceInterfaceModule.get("cdi");

        colorSensor = new ColorNumberSensor();
        colorSensor.init(ahwMap.i2cDevice.get("cc"));
        distance = ahwMap.ultrasonicSensor.get("distance");
    }



    public void setMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
    }

    public void setTargetPosition(double left, double right) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + (int) (left * COUNTS_PER_INCH));
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + (int) (right * COUNTS_PER_INCH));
     }

    public boolean isDriving() {
        return (frontLeftMotor.isBusy() && frontRightMotor.isBusy());
    }

    // Power the left and right wheels as needed
    public void drive(double left, double right) {
        frontLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
    }

    public void redLED(boolean state) {
        cdi.setLED(1, state);
    }

    public void blueLED(boolean state) {
        cdi.setLED(0, state);
    }

}

