package com.github.pmtischler.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tank Drive controls for Robot.
 */
@TeleOp(name="pmtischler.TankDrive", group="pmtischler")
public class TankDrive extends RobotHardware {

    /**
     * Tank drive control program.
     */
    public void loop() {
        setDriveForTank(gamepad1.left_stick_y, gamepad1.right_stick_y);
    }
}
