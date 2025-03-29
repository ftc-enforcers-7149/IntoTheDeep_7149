package org.firstinspires.ftc.teamcode.Hardware.CoreMotion.EncoderResets;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CurrentResetter implements EncoderReset {

    private double currentLimit;
    private CurrentUnit unit;

    public CurrentResetter(double limit, CurrentUnit unit) {
        currentLimit = limit;
        this.unit = unit;
    }

    @Override
    public boolean attemptReset(DcMotorEx motor) {

        return (motor.getCurrent(unit) > currentLimit);

    }
}
