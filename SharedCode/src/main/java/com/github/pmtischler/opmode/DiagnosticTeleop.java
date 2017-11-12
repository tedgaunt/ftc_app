package com.github.pmtischler.opmode;

import com.github.pmtischler.base.Color;
import com.github.pmtischler.vision.SimpleVuforia;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Diagnostic teleop mode.
 * This mode emits telemetry for use as diagnostic.
 */
@TeleOp(name="pmt.Diagnostic", group="pmtischler")
public class DiagnosticTeleop extends RelicRecoveryManual {
    /**
     * Extends teleop initialization to start a recorder.
     */
    public void init() {
        super.init();

        vuforia = new SimpleVuforia(getVuforiaLicenseKey());
        setColorSensorLedEnabled(ColorSensorName.JEWEL, true);
    }

    /**
     * Extends teleop control to record hardware after loop.
     */
    public void loop() {
        super.loop();

        telemetry.addData("Jewel Red", getColorSensor(
                    ColorSensorName.JEWEL, Color.Channel.RED));
        telemetry.addData("Jewel Blue", getColorSensor(
                    ColorSensorName.JEWEL, Color.Channel.BLUE));

        telemetry.addData("Distance Left (cm)", getDistanceSensorCm(
                    DistanceSensorName.LEFT));
        telemetry.addData("Distance Front (cm)", getDistanceSensorCm(
                    DistanceSensorName.FRONT));
        telemetry.addData("Distance Right (cm)", getDistanceSensorCm(
                    DistanceSensorName.RIGHT));

        telemetry.addData("Vuforia", vuforia.detectMark().name());
    }

    private SimpleVuforia vuforia;
}
