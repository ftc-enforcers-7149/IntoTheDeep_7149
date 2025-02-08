package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

public class AxonAbsoluteEncoder implements PeriodicAction {

    private AnalogInput analogInput;
    private double absolutePosition;
    private double currentPosition, lastPosition;
    private double offset;
    private final double CONVERT_DEG = 3.3/360.0;

    public AxonAbsoluteEncoder(HardwareMap hMap, String name) {
        analogInput = hMap.get(AnalogInput.class, name);
        currentPosition = analogInput.getVoltage() * CONVERT_DEG;
        lastPosition = currentPosition;
        absolutePosition = currentPosition;

        offset = 0;
    }


    @Override
    public void periodic() {

        currentPosition = analogInput.getVoltage() * CONVERT_DEG;

        //wrap around from 0 -> 360 (go backward)
        if (currentPosition > 300 && lastPosition < 60) {
            absolutePosition += -1 * (lastPosition + (360 - currentPosition));

            //wrap around from 360 -> 0
        } else if (currentPosition < 0 && lastPosition > 300) {
            absolutePosition += (360 - lastPosition) + currentPosition;
        } else {
            absolutePosition += currentPosition - lastPosition;
        }

        lastPosition = currentPosition;

    }

    public double getAbsolutePosition() {
        return absolutePosition - offset;
    }

    /**
     * Sets the current position to the input offset value. Future calls to getAbsolutePosition()
     * will reflect the new offset.
     * @param offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }
}
