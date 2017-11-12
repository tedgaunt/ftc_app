package com.github.pmtischler.opmode;

import com.github.pmtischler.R;
import com.github.pmtischler.base.Color;
import com.github.pmtischler.base.StateMachine;
import com.github.pmtischler.base.StateMachine.State;
import com.github.pmtischler.control.Mecanum;
import com.github.pmtischler.vision.SimpleVuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Autonomous demo for FTC Relic Recovery game.
 */
public class RelicRecoveryAuto extends RobotHardware {

    @Autonomous(name="pmt.Red.Center", group="pmtischler")
    public static class RelicRecoveryAutoRedCenter extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Red.Corner", group="pmtischler")
    public static class RelicRecoveryAutoRedCorner extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Blue.Center", group="pmtischler")
    public static class RelicRecoveryAutoBlueCenter extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Blue.Corner", group="pmtischler")
    public static class RelicRecoveryAutoBlueCorner extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();

        driveOffSec = hardwareMap.appContext.getResources().getInteger(
                R.integer.drive_off_ms) / 1000.0;
        turnTowardSec = hardwareMap.appContext.getResources().getInteger(
                R.integer.turn_toward_ms) / 1000.0;

        vuMark = RelicRecoveryVuMark.UNKNOWN;

        telemetry.addData("Robot Color", robotColor.name());
        telemetry.addData("Robot Start Position", robotStartPos.name());

        StateMachine.State driveToCryptobox = newDriveToCryptobox(null);
        StateMachine.State hitJewel = newHitJewel(driveToCryptobox);
        StateMachine.State detectVuforia = new DetectVuforia(hitJewel);

        machine = new StateMachine(detectVuforia);

        telemetry.update();
    }

    @Override
    public void loop() {
        machine.update();
        telemetry.addData("vuMark", vuMark.name());
        telemetry.update();
    }

    // State in the machine to wait for a duration.
    private class WaitForDuration implements StateMachine.State {
        public WaitForDuration(double duration, StateMachine.State next) {
            this.duration = duration;
            this.next = next;
        }

        @Override
        public void start() {
            startTime = time;
        }

        @Override
        public State update() {
            if (time - startTime > duration) {
                return next;
            }
            return this;
        }

        private double duration;
        private StateMachine.State next;
        private double startTime;
    }

    // Detects the Vuforia Mark.
    private class DetectVuforia implements StateMachine.State {
        public DetectVuforia(StateMachine.State next) {
            this.next = next;
            vuforia = new SimpleVuforia(getVuforiaLicenseKey());
        }

        @Override
        public void start() {
            startTime = time;
        }

        @Override
        public State update() {
            vuMark = vuforia.detectMark();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                // Found the mark.
                return next;
            }

            if (time - startTime > 5) {
                // Did not detect mark within time bound.
                vuMark = RelicRecoveryVuMark.UNKNOWN;
                return next;
            }

            return this;
        }

        private StateMachine.State next;
        private SimpleVuforia vuforia;
        private double startTime;
    }


    // Drops the jewel arm.
    private class DropJewelArm implements StateMachine.State {
        public DropJewelArm(StateMachine.State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            setColorSensorLedEnabled(ColorSensorName.JEWEL, true);
            lowerJewelArm();
            return new WaitForDuration(2, next);
        }

        private StateMachine.State next;
    }

    // Reads the jewel color.
    private class HitJewel implements StateMachine.State {
        public HitJewel(StateMachine.State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            int r = getColorSensor(ColorSensorName.JEWEL,
                                   Color.Channel.RED);
            int b = getColorSensor(ColorSensorName.JEWEL,
                                   Color.Channel.BLUE);

            if ((r > b && robotColor == Color.Ftc.BLUE) ||
                    (b > r && robotColor == Color.Ftc.RED)) {
                // Reading other team's jewel in forward position.
                forwardJewelArm();
            } else {
                // Reading our team's jewel in forward position.
                backwardJewelArm();
            }
            setColorSensorLedEnabled(ColorSensorName.JEWEL, false);
            return new WaitForDuration(1, next);
        }

        private StateMachine.State next;
    }

    // Resets the jewel arm to the starting position.
    private class ResetJewelArm implements StateMachine.State {
        public ResetJewelArm(StateMachine.State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            raiseJewelArm();
            centerJewelArm();
            return new WaitForDuration(1, next);
        }

        private StateMachine.State next;
    }

    private StateMachine.State newHitJewel(StateMachine.State next) {
        StateMachine.State jewelReset = new ResetJewelArm(next);
        StateMachine.State jewelHit = new HitJewel(jewelReset);
        StateMachine.State jewelDrop = new DropJewelArm(jewelHit);
        return jewelDrop;
    }

    // Drives a specific motion for a specific amount of time.
    private class DriveForTime implements StateMachine.State {
        public DriveForTime(Mecanum.Motion motion, double duration,
                                  StateMachine.State next) {
            this.motion = motion;
            this.duration = duration;
            this.next = next;
        }

        @Override
        public void start() {
            startTime = time;
        }

        @Override
        public State update() {
            if (time - startTime < duration) {
                setDriveForMecanum(motion);
                return this;
            } else {
                setDriveForMecanum(new Mecanum.Motion(0, 0, 0));
                return next;
            }
        }

        private double startTime;
        private Mecanum.Motion motion;
        private double duration;
        private StateMachine.State next;
    }

    private StateMachine.State newDriveToCryptobox(StateMachine.State next) {
        StateMachine.State turnToFace;
        if (robotColor == Color.Ftc.RED) {
            if (robotStartPos == StartPosition.FIELD_CENTER) {
                // Red center does not turn.
                turnToFace = next;
            } else {
                // Red corner turns right.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, -0.5),
                        turnTowardSec, next);
            }
        } else {
            if (robotStartPos == StartPosition.FIELD_CENTER) {
                // Blue center turns around.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, 0.5),
                        turnTowardSec * 2, next);
            } else {
                // Blue corner turns right.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, -0.5),
                        turnTowardSec, next);
            }
        }

        // Red drives off plaform forward, blue backwards.
        double driveOffAngle;
        if (robotColor == Color.Ftc.RED) {
            driveOffAngle = 0;
        } else {
            driveOffAngle = Math.PI;
        }
        StateMachine.State driveOff = new DriveForTime(
                new Mecanum.Motion(0.5, driveOffAngle, 0),
                driveOffSec, turnToFace);
        return driveOff;
    }

    // The state machine.
    private StateMachine machine;
    // The robot's color.
    protected Color.Ftc robotColor;
    // The robot's starting position.
    protected StartPosition robotStartPos;
    // The detected Vuforia Mark.
    private RelicRecoveryVuMark vuMark;

    // Per robot tuning parameters.
    // Seconds to drive off platform and turn.
    private double driveOffSec;
    private double turnTowardSec;
}
