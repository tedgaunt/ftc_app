package com.github.pmtischler.opmode;

import com.github.pmtischler.base.StateMachine;
import com.github.pmtischler.base.StateMachine.State;
import com.github.pmtischler.control.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Autonomous demo for FTC Relic Recovery game.
 */
@Autonomous(name="pmtischler.RelicRecoveryAuto", group="pmtischler")
public class RelicRecoveryAuto extends RobotHardware {

    @Override
    public void init() {
        super.init();

        StateMachine.State jewel_reset = new ResetJewelArm(null);
        StateMachine.State jewel_hit_wait = new WaitForDuration(2, jewel_reset);
        StateMachine.State jewel_hit = new HitJewel(jewel_hit_wait);
        StateMachine.State jewel_drop_wait = new WaitForDuration(2, jewel_hit);
        StateMachine.State jewel_drop = new DropJewelArm(jewel_drop_wait);

        machine = new StateMachine(jewel_drop);
    }

    @Override
    public void loop() {

    }

    // The state machine.
    private StateMachine machine;

    // State in the machine to wait for a duration.
    public class WaitForDuration implements StateMachine.State {
        public WaitForDuration(double duration, StateMachine.State next) {
            this.duration = duration;
            this.next = next;
        }

        @Override
        public void start() {
            start_time = time;
        }

        @Override
        public State update() {
            if (time - start_time > duration) {
                return next;
            }
            return this;
        }

        private double duration;
        private StateMachine.State next;
        private double start_time;
    }

    // Drops the jewel arm.
    public class DropJewelArm implements StateMachine.State {
        public DropJewelArm(StateMachine.State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            lowerJewelArm();
            return next;
        }

        private StateMachine.State next;
    }

    // Reads the jewel color.
    public class HitJewel implements StateMachine.State {
        public HitJewel(StateMachine.State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            int red = getColorSensor(ColorSensorName.JEWEL, Color.RED);
            int blue = getColorSensor(ColorSensorName.JEWEL, Color.BLUE);

            if (red > blue) {
                forwardJewelArm();
            } else {
                backwardJewelArm();
            }
            return next;
        }

        private StateMachine.State next;
    }

    // Resets the jewel arm to the starting position.
    public class ResetJewelArm implements StateMachine.State {
        public ResetJewelArm(StateMachine.State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            raiseJewelArm();
            centerJewelArm();
            return next;
        }

        private StateMachine.State next;
    }
}
