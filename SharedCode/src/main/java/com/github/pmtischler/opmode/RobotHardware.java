package com.github.pmtischler.opmode;

import com.github.pmtischler.control.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

/**
 * Hardware Abstraction Layer for Robot.
 * Provides common variables and functions for the hardware.
 * The robot configuration in the app should match enum names.
 */
public abstract class RobotHardware extends OpMode {
    // The motors on the robot.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
    }

    /**
     * Sets the power of the motor.
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    public void setPower(MotorName motor, double power) {
        allMotors.get(motor.ordinal()).setPower(power);
    }

    /**
     * Sets the drive chain power.
     * @param left The power for the left two motors.
     * @param right The power for the right two motors.
     */
    public void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * @param motion The desired Mecanum motion.
     */
    public void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.frontRight);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    // The servos on the robot.
    public enum ServoName {
        JEWEL_DROP,
        JEWEL_HIT,
    }

    /**
     * Sets the angle of the servo.
     * @param servo The servo to modify.
     * @param position The angle to set [0, 1].
     */
    public void setAngle(ServoName servo, double position) {
        allServos.get(servo.ordinal()).setPosition(position);
    }

    // Raises the jewel arm.
    public void raiseJewelArm() {
        setAngle(ServoName.JEWEL_DROP, 1);
    }

    // Lowers the jewel arm.
    public void lowerJewelArm() {
        setAngle(ServoName.JEWEL_DROP, 0);
    }

    // Centers the jewel arm.
    public void centerJewelArm() {
        setAngle(ServoName.JEWEL_HIT, 0.5);
    }

    // Moves the jewel arm forward.
    public void forwardJewelArm() {
        setAngle(ServoName.JEWEL_HIT, 1);
    }

    // Moves the jewel arm backward.
    public void backwardJewelArm() {
        setAngle(ServoName.JEWEL_HIT, 0);
    }

    // Possible colors.
    public enum Color {
        RED,
        GREEN,
        BLUE,
        ALPHA,
    }

    // The color sensors on the robot.
    public enum ColorSensorName {
        JEWEL,
    }

    /**
     * Gets the color value on the sensor.
     * @param sensor The sensor to read.
     * @param color The color to read intensity.
     */
    public int getColorSensor(ColorSensorName sensor, Color color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        switch (color) {
            case RED: return s.red();
            case GREEN: return s.green();
            case BLUE: return s.blue();
            case ALPHA: return s.alpha();
            default: return 0;
        }
    }

    /**
     * Initialize the hardware handles.
     */
    public void init() {
        allMotors = new ArrayList<DcMotor>();
        for (MotorName m : MotorName.values()) {
            DcMotor motor = hardwareMap.get(DcMotor.class, m.name());
            motor.setPower(0);
            allMotors.add(motor);
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            Servo servo = hardwareMap.get(Servo.class, s.name());
            allServos.add(servo);
        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            ColorSensor sensor = hardwareMap.get(ColorSensor.class, s.name());
            sensor.enableLed(true);
            allColorSensors.add(sensor);
        }

        raiseJewelArm();
    }

    /**
     * End of match, stop all actuators.
     */
    public void stop() {
        super.stop();

        for (DcMotor motor : allMotors) {
            motor.setPower(0);
        }
        for (ColorSensor sensor : allColorSensors) {
            sensor.enableLed(false);
        }
    }

    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;
    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;
    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;
}
