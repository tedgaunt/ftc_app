package com.github.pmtischler.opmode;

import com.github.pmtischler.R;
import com.github.pmtischler.base.Color;
import com.github.pmtischler.base.StateMachine;
import com.github.pmtischler.base.StateMachine.State;
import com.github.pmtischler.control.Mecanum;
import com.github.pmtischler.control.Pid;
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
        StateMachine.State driveToColumn = new NavigateViaDistance(next);

        StateMachine.State turnToFace;
        if (robotColor == Color.Ftc.RED) {
            if (robotStartPos == StartPosition.FIELD_CENTER) {
                // Red center does not turn.
                turnToFace = next;
            } else {
                // Red corner turns right.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, -0.5),
                        turnTowardSec, driveToColumn);
            }
        } else {
            if (robotStartPos == StartPosition.FIELD_CENTER) {
                // Blue center turns around.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, 0.5),
                        turnTowardSec * 2, driveToColumn);
            } else {
                // Blue corner turns right.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, -0.5),
                        turnTowardSec, driveToColumn);
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

    // Drives towards a position on the map using distance sensor readings.
    // Assumes the robot is in the correct orientation and square with the field.
    private class NavigateViaDistance implements StateMachine.State {
        public NavigateViaDistance(StateMachine.State next) {
            this.next = next;

            targetFrontDistCm = cryptoFrontDistCm;
            targetSideDistCm = cryptoSideDistCm;
            if (robotColor == Color.Ftc.RED) {
                sideSensor = DistanceSensorName.RIGHT;
            } else {
                sideSensor = DistanceSensorName.LEFT;
            }

            double kp = 0.5 / 20.0;    // 50% power at 20cm.
            double ti = 1.0;           // 1s to eliminate past errors.
            double td = 0.1;           // 0.1s to predict future error.
            double integralMax = 1.0;  // Clamp integral at 100% of power.
            frontPid = new Pid(kp, ti, td, -integralMax, integralMax);
            sidePid = new Pid(kp, ti, td, -integralMax, integralMax);
        }

        @Override
        public void start() {
            lastTime = time;
            lastTimeOutsideRange = time;
        }

        @Override
        public State update() {
            double dt = time - lastTime;
            double frontCm = getDistanceSensorCm(DistanceSensorName.FRONT);
            double sideCm = getDistanceSensorCm(sideSensor);
            double totalCm = Math.sqrt(
                    Math.pow(frontCm, 2) + Math.pow(sideCm, 2));

            telemetry.addData("Front Dist (cm)", frontCm);
            telemetry.addData("Side Dist (cm)", sideCm);
            telemetry.addData("Total Dist (cm)", totalCm);

            if (totalCm > targetSatisfyDistCm) {
                lastTimeOutsideRange = time;
            }
            if (time - lastTimeOutsideRange >= 2.0) {
                // Settled in target position, done navigating.
                setDriveForMecanum(new Mecanum.Motion(0, 0, 0));
                return next;
            }

            double frontPower = -frontPid.update(targetFrontDistCm, frontCm, dt);
            double sidePower = -sidePid.update(targetSideDistCm, sideCm, dt);
            if (sideSensor == DistanceSensorName.RIGHT) {
                // Right sensor faces -Y.
                sidePower = sidePower * -1;
            }
            double totalPower = Math.sqrt(
                    Math.pow(frontPower, 2) + Math.pow(sidePower, 2));
            double powerAngle = Math.atan2(sidePower, frontPower);

            setDriveForMecanum(new Mecanum.Motion(totalPower, powerAngle, 0));

            lastTime = time;
            return this;
        }

        private StateMachine.State next;

        // Dist from target where it's considered satisfied.
        private double targetSatisfyDistCm = 5;
        // Last iteration time for dt.
        private double lastTime;
        // Last time out of target range.
        private double lastTimeOutsideRange;
        // Target distance readings.
        private double targetFrontDistCm;
        private double targetSideDistCm;
        // Side sensor name.
        private DistanceSensorName sideSensor;

        private Pid frontPid;
        private Pid sidePid;
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
    // Target distance readings to navigate to cryptobox.
    // TODO: Support 12 values (4 start pos, 3 goals).
    // TODO: Load this from res.
    private double cryptoFrontDistCm = 30;
    private double cryptoSideDistCm = 100;
}
