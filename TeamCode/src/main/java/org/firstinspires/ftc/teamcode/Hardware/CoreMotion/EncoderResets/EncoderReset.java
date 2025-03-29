package org.firstinspires.ftc.teamcode.Hardware.CoreMotion.EncoderResets;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public interface EncoderReset {

    public boolean attemptReset(DcMotorEx motor);

}
