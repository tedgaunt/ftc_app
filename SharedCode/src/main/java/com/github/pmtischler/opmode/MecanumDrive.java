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
     * Mecanum drive control program.
     */
    public void loop() {
        setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y));
    }
}
