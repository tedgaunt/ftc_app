package com.github.pmtischler.opmode;

import com.github.pmtischler.base.Color;
import com.github.pmtischler.base.StateMachine;
import com.github.pmtischler.base.StateMachine.State;
import com.github.pmtischler.control.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Autonomous demo for FTC Relic Recovery game.
 */
public class RelicRecoveryAuto extends RobotHardware {

    @Autonomous(name="pmt.Auto.Red.Center", group="pmtischler")
    public static class RelicRecoveryAutoRedCenter extends RelicRecoveryAuto {
        @Override public void init() {
            robot_color = Color.Ftc.RED;
            robot_start_pos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Auto.Red.Corner", group="pmtischler")
    public static class RelicRecoveryAutoRedCorner extends RelicRecoveryAuto {
        @Override public void init() {
            robot_color = Color.Ftc.RED;
            robot_start_pos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Auto.Blue.Center", group="pmtischler")
    public static class RelicRecoveryAutoBlueCenter extends RelicRecoveryAuto {
        @Override public void init() {
            robot_color = Color.Ftc.BLUE;
            robot_start_pos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Auto.Blue.Corner", group="pmtischler")
    public static class RelicRecoveryAutoBlueCorner extends RelicRecoveryAuto {
        @Override public void init() {
            robot_color = Color.Ftc.BLUE;
            robot_start_pos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();

        telemetry.addData("Robot Color", robot_color.name());
        telemetry.addData("Robot Start Position", robot_start_pos.name());

        StateMachine.State jewel_reset = new ResetJewelArm(null);
        StateMachine.State jewel_hit = new HitJewel(jewel_reset);
        StateMachine.State jewel_drop = new DropJewelArm(jewel_hit);

        machine = new StateMachine(jewel_drop);

        telemetry.update();
    }

    @Override
    public void loop() {
        machine.update();
        telemetry.update();
    }

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
            return new WaitForDuration(2, next);
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
            int r = getColorSensor(ColorSensorName.JEWEL,
                                   Color.Channel.RED);
            int b = getColorSensor(ColorSensorName.JEWEL,
                                   Color.Channel.BLUE);

            if ((r > b && robot_color == Color.Ftc.BLUE) ||
                    (b > r && robot_color == Color.Ftc.RED)) {
                // Reading other team's jewel in forward position.
                forwardJewelArm();
            } else {
                // Reading our team's jewel in forward position.
                backwardJewelArm();
            }
            return new WaitForDuration(1, next);
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
            return new WaitForDuration(1, next);
        }

        private StateMachine.State next;
    }

    // The state machine.
    private StateMachine machine;
    // The robot's color.
    protected Color.Ftc robot_color;
    // The robot's starting position.
    protected StartPosition robot_start_pos;
}
