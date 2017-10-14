package com.github.pmtischler.control;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

/**
 * Tests correctness of Mecanum calculations.
 */
public class MecanumTest {
    // The comparison threshold.
    private static final double diffThresh = 0.001;

    /**
     * Asserts that the input controls yields the expected motion.
     */
    private void assertMotion(double left_stick_x, double left_stick_y,
                              double right_stick_x, double right_stick_y,
                              double vD, double thetaD, double vTheta) {
        Mecanum.Motion motion = Mecanum.joystickToMotion(
                left_stick_x, left_stick_y,
                right_stick_x, right_stick_y);
        assertEquals(vD, motion.vD, diffThresh);
        assertEquals(thetaD, motion.thetaD, diffThresh);
        assertEquals(vTheta, motion.vTheta, diffThresh);
    }

    @Test
    // Test Mecanum for direct strafing.
    public void testMecanumMotionStrafing() throws Exception {
        // Forward.
        assertMotion(0, 1,
                     0, 0,
                     1, 0, 0);
        // Right.
        assertMotion(1, 0,
                     0, 0,
                     1, Math.PI / 2, 0);
        // Back.
        assertMotion(0, -1,
                     0, 0,
                     1, Math.PI, 0);
        // Left.
        assertMotion(-1, 0,
                     0, 0,
                     1, - Math.PI / 2, 0);

        // Front right.
        assertMotion(1, 1,
                     0, 0,
                     Math.sqrt(2), Math.PI / 4, 0);
    }

    @Test
    // Test Mecanum for turning.
    public void testMecanumMotionTurning() throws Exception {
        // Left.
        assertMotion(0, 0,
                     -1, 0,
                     0, 0, 1);
        // Right.
        assertMotion(0, 0,
                     1, 0,
                     0, 0, -1);
    }

    /**
     * Asserts that the input motion yields the expected wheel poweers.
     */
    private void assertWheels(double vD, double thetaD, double vTheta,
                              double frontLeft, double frontRight,
                              double backLeft, double backRight) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(
                new Mecanum.Motion(vD, thetaD, vTheta));
        assertEquals(frontLeft, wheels.frontLeft, diffThresh);
        assertEquals(frontRight, wheels.frontRight, diffThresh);
        assertEquals(backLeft, wheels.backLeft, diffThresh);
        assertEquals(backRight, wheels.backRight, diffThresh);
    }

    @Test
    // Test Mecanum for direct strafing.
    public void testMecanumWheelStrafing() throws Exception {
        // Forward.
        assertWheels(1, 0, 0,
                     0.7071, 0.7071,
                     0.7071, 0.7071);
        // Right.
        assertWheels(1, Math.PI / 2, 0,
                     0.7071, -0.7071,
                     -0.7071, 0.7071);
        // Back.
        assertWheels(1, Math.PI, 0,
                     -0.7071, -0.7071,
                     -0.7071, -0.7071);
        // Left.
        assertWheels(1, 3 * Math.PI / 2, 0,
                     -0.7071, 0.7071,
                     0.7071, -0.7071);

        // Front right.
        assertWheels(1, Math.PI / 4, 0,
                     1, 0,
                     0, 1);
    }

    @Test
    // Test Mecanum for turning.
    public void testMecanumWheelTurning() throws Exception {
        // Right.
        assertWheels(0, 0, 1,
                     1, -1,
                     1, -1);
        // Left.
        assertWheels(0, 0, -1,
                     -1, 1,
                     -1, 1);
    }

    @Test
    // Test Mecanum for moving and turning to clamp motors.
    public void testMecanumWheelClamping() throws Exception {
        // Forward and full right.
        assertWheels(1, 0, 1,
                     1, -0.1716,
                     1, -0.1716);
    }
}
