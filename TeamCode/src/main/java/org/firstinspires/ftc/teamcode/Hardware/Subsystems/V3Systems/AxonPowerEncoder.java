package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonPowerEncoder {

    private AnalogInput analogInput;
    private double absolutePosition;
    private double currentPosition, lastPosition;
    private double offset;
    private final double CONVERT_DEG = 360.0/3.3;

    private boolean IN_WRAPAROUND = false;

    public AxonPowerEncoder(OpMode opmode, String name) {
        analogInput = opmode.hardwareMap.get(AnalogInput.class, name);
        currentPosition = analogInput.getVoltage() * CONVERT_DEG;
        lastPosition = currentPosition;
        absolutePosition = currentPosition;

        offset = 0;
    }


    public void update(double power) {

        currentPosition = analogInput.getVoltage() * CONVERT_DEG;

//        //wrap around from 0 -> 360 (go backward)
        if ( power < -0.02 && (currentPosition < 115 && lastPosition > 225)) {
            absolutePosition += (360 - lastPosition) + currentPosition;
            //IN_WRAPAROUND = true;
        } else if ( power > 0.02 && (currentPosition > 225 && lastPosition < 115)) {
            absolutePosition += -1 * (lastPosition + (360 - currentPosition));
        } else {
            absolutePosition += currentPosition - lastPosition;
        }


        //wrap around from 0 -> 360 (go backward)
//        if ( power < 0 && (currentPosition < lastPosition - 10)) {
//            absolutePosition += (360 - lastPosition) + currentPosition;
//        } else if ( power > 0 && (currentPosition > lastPosition + 10)) {
//            absolutePosition += -1 * (lastPosition + (360 - currentPosition));
//        } else {
//            absolutePosition += currentPosition - lastPosition;
//        }

        lastPosition = currentPosition;

    }

    public double getAbsolutePosition() {
        return absolutePosition - offset;
    }

    public double getCurrentPosition() { return currentPosition; };

    /**
     * Sets the current position to the input offset value. Future calls to getAbsolutePosition()
     * will reflect the new offset.
     * @param offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }
}
