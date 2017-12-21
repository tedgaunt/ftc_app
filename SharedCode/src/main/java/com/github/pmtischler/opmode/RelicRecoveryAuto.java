package com.github.pmtischler.opmode;

import com.github.pmtischler.R;
import com.github.pmtischler.base.BlackBox;
import com.github.pmtischler.base.Color;
import com.github.pmtischler.base.StateMachine;
import com.github.pmtischler.base.StateMachine.State;
import com.github.pmtischler.control.Mecanum;
import com.github.pmtischler.control.Pid;
import com.github.pmtischler.vision.SimpleVuforia;

import android.content.res.Resources;
import android.util.TypedValue;
import java.io.FileInputStream;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Autonomous demo for FTC Relic Recovery game.
 */
public class RelicRecoveryAuto extends RobotHardware {

    @Autonomous(name="pmt.Red.Center", group="pmtischler")
    public static class RedCenter extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Red.Corner", group="pmtischler")
    public static class RedCorner extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Blue.Center", group="pmtischler")
    public static class BlueCenter extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Blue.Corner", group="pmtischler")
    public static class BlueCorner extends RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        vuMark = RelicRecoveryVuMark.UNKNOWN;

        driveMaxPower = hardwareMap.appContext.getResources().getInteger(
                R.integer.drive_max_power_percent) / 100.0;

        telemetry.addData("Robot Color", robotColor.name());
        telemetry.addData("Robot Start Position", robotStartPos.name());

        StateMachine.State scoreGlyph = new BlackboxPlayback(
                getStartPositionName(robotColor, robotStartPos), null);
        StateMachine.State driveToCryptobox = newDriveToCryptobox(scoreGlyph);
        StateMachine.State hitJewel = newHitJewel(driveToCryptobox);
        StateMachine.State detectVuforia = new DetectVuforia(hitJewel);
        machine = new StateMachine(detectVuforia);

        telemetry.update();
    }

    @Override
    public void loop() {
        super.loop();
        machine.update();
        telemetry.addData("vuMark", vuMark.name());
        telemetry.update();
    }

    double getResourceDouble(int id) {
        TypedValue outValue = new TypedValue();
        hardwareMap.appContext.getResources().getValue(id, outValue, true);
        return outValue.getFloat();
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

        Resources res = hardwareMap.appContext.getResources();
        double driveOffSec;
        double driveOffAngle;
        double turnTowardSec;

        StateMachine.State turnToFace;
        if (robotColor == Color.Ftc.RED) {
            // Red drives forward off platform.
            driveOffAngle = 0;
            if (robotStartPos == StartPosition.FIELD_CENTER) {
                driveOffSec  = res.getInteger(
                        R.integer.red_center_drive_off_ms) / 1000.0;
                // Red center does not turn.
                turnToFace = driveToColumn;
            } else {
                driveOffSec  = res.getInteger(
                        R.integer.red_corner_drive_off_ms) / 1000.0;
                turnTowardSec = res.getInteger(
                        R.integer.red_corner_turn_toward_ms) / 1000.0;
                // Red corner turns right.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, -driveMaxPower),
                        turnTowardSec, driveToColumn);
            }
        } else {
            // Blue drives backwards off platform.
            driveOffAngle = Math.PI;
            if (robotStartPos == StartPosition.FIELD_CENTER) {
                driveOffSec  = res.getInteger(
                        R.integer.blue_center_drive_off_ms) / 1000.0;
                turnTowardSec = res.getInteger(
                        R.integer.blue_center_turn_toward_ms) / 1000.0;
                // Blue center turns around.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, driveMaxPower),
                        turnTowardSec, driveToColumn);
            } else {
                driveOffSec  = res.getInteger(
                        R.integer.blue_corner_drive_off_ms) / 1000.0;
                turnTowardSec = res.getInteger(
                        R.integer.blue_corner_turn_toward_ms) / 1000.0;
                // Blue corner turns right.
                turnToFace = new DriveForTime(
                        new Mecanum.Motion(0, 0, -driveMaxPower),
                        turnTowardSec, driveToColumn);
            }
        }
        StateMachine.State driveOff = new DriveForTime(
                new Mecanum.Motion(driveMaxPower, driveOffAngle, 0),
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

            {
                double kp = getResourceDouble(R.dimen.nav_translate_pid_kp);
                double ti = getResourceDouble(R.dimen.nav_translate_pid_ti);
                double td = getResourceDouble(R.dimen.nav_translate_pid_td);
                double integralMax = getResourceDouble(
                        R.dimen.nav_translate_pid_integral_max);
                frontPid = new Pid(kp, ti, td,
                                   -integralMax, integralMax,
                                   -driveMaxPower, driveMaxPower);
                sidePid = new Pid(kp, ti, td,
                                  -integralMax, integralMax,
                                  -driveMaxPower, driveMaxPower);
            }
            {
                double kp = getResourceDouble(R.dimen.nav_rotate_pid_kp);
                double ti = getResourceDouble(R.dimen.nav_rotate_pid_ti);
                double td = getResourceDouble(R.dimen.nav_rotate_pid_td);
                double integralMax = getResourceDouble(
                        R.dimen.nav_rotate_pid_integral_max);
                rotatePid = new Pid(kp, ti, td,
                                    -integralMax, integralMax,
                                    -driveMaxPower, driveMaxPower);
            }
        }

        @Override
        public void start() {
            lastTime = time;
            lastTimeOutsideRange = time;
        }

        @Override
        public State update() {
            double dt = time - lastTime;
            double frontLeftCm = getDistanceSensorCm(
                    DistanceSensorName.FRONT_LEFT);
            double frontRightCm = getDistanceSensorCm(
                    DistanceSensorName.FRONT_RIGHT);
            double frontCm = (frontLeftCm + frontRightCm) / 2.0;
            double sideCm = getDistanceSensorCm(sideSensor);
            double errorDistCm = Math.sqrt(
                    Math.pow(frontCm - targetFrontDistCm, 2) +
                    Math.pow(sideCm - targetSideDistCm, 2));
            double errorRotateDistCm = Math.abs(frontLeftCm - frontRightCm);

            telemetry.addData("Front Left Dist (cm)", frontLeftCm);
            telemetry.addData("Front Right Dist (cm)", frontRightCm);
            telemetry.addData("Front Avg Dist (cm)", frontCm);
            telemetry.addData("Side Dist (cm)", sideCm);
            telemetry.addData("Error Dist (cm)", errorDistCm);
            telemetry.addData("Error Rotate Dist (cm)", errorRotateDistCm);

            // Check if the robot has stayed at goal for required time.
            if (errorDistCm > distDiffSatisfyCm ||
                    errorRotateDistCm > rotateDiffSatisfyCm) {
                lastTimeOutsideRange = time;
            }
            if (time - lastTimeOutsideRange >= 2.0) {
                // Settled in target position, done navigating.
                setDriveForMecanum(new Mecanum.Motion(0, 0, 0));
                return next;
            }

            // Determine translate motor powers.
            double frontPower = -frontPid.update(targetFrontDistCm, frontCm, dt);
            double sidePower = -sidePid.update(targetSideDistCm, sideCm, dt);
            if (sideSensor == DistanceSensorName.RIGHT) {
                // Right sensor faces -Y.
                sidePower = sidePower * -1;
            }
            double translatePower = Math.sqrt(
                    Math.pow(frontPower, 2) + Math.pow(sidePower, 2));
            double powerAngle = Math.atan2(sidePower, frontPower);

            // Determine rotation motor powers.
            double rotatePower = -rotatePid.update(frontLeftCm, frontRightCm, dt);

            setDriveForMecanum(new Mecanum.Motion(
                        translatePower, powerAngle, rotatePower));

            lastTime = time;
            return this;
        }

        private StateMachine.State next;

        // Dist from target where it's considered satisfied.
        private double distDiffSatisfyCm = 3;
        private double rotateDiffSatisfyCm = 3;
        // Last iteration time for dt.
        private double lastTime;
        // Last time out of target range.
        private double lastTimeOutsideRange;
        // Target distance readings.
        private double targetFrontDistCm;
        private double targetSideDistCm;
        // Side sensor name.
        private DistanceSensorName sideSensor;

        // Control loops for navigating.
        private Pid frontPid;
        private Pid sidePid;
        private Pid rotatePid;
    }

    // Plays back a recorded blackbox stream.
    private class BlackboxPlayback implements StateMachine.State {
        public BlackboxPlayback(String filename, StateMachine.State next) {
            try {
                inputStream = hardwareMap.appContext.openFileInput(filename);
                player = new BlackBox.Player(inputStream, hardwareMap);
            } catch (Exception e) {
                e.printStackTrace();
                requestOpModeStop();
            }
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            try {
                if (player.playback(time)) {
                    return this;
                } else {
                    return next;
                }
            } catch (Exception e) {
                e.printStackTrace();
                requestOpModeStop();
                return null;
            }
        }

        private StateMachine.State next;
        // The input file stream.
        private FileInputStream inputStream;
        // The hardware player.
        private BlackBox.Player player;
    }

    // The state machine.
    private StateMachine machine;
    // The robot's color.
    protected Color.Ftc robotColor;
    // The robot's starting position.
    protected StartPosition robotStartPos;
    // The detected Vuforia Mark.
    private RelicRecoveryVuMark vuMark;

    // Max output power on driving motors.
    private double driveMaxPower;

    // Target distance readings to navigate to cryptobox.
    // TODO: Support 12 values (4 start pos, 3 goals).
    // TODO: Load this from res.
    private double cryptoFrontDistCm = 30;
    private double cryptoSideDistCm = 30;
}
