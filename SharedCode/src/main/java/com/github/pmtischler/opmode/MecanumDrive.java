package com.github.pmtischler.opmode;

import com.github.pmtischler.control.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Mecanum Drive controls for Robot.
 */
@TeleOp(name="pmtischler.MecanumDrive", group="pmtischler")
public class MecanumDrive extends RobotHardware {
    /**
     * Sets the drive chain power.
     * @param motion The desired Mecanum motion.
     */
    public void setDrive(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.frontRight);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    /**
     * Mecanum drive control program.
     */
    public void loop() {
        setDrive(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y));
    }
}
