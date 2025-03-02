package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class FourServoPitchArm {

    private ServoImplEx left1, left2, right1, right2;

    private double lastPos = -999;

    public FourServoPitchArm(OpMode opMode) {

        left1 = opMode.hardwareMap.get(ServoImplEx.class, "left1");
        left2 = opMode.hardwareMap.get(ServoImplEx.class, "left2");
        right1 = opMode.hardwareMap.get(ServoImplEx.class, "right1");
        right2 = opMode.hardwareMap.get(ServoImplEx.class, "right2");

        left1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        left2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right2.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setPosition(double pos) {

        if (Math.abs(pos - lastPos) > 0.001) {

            left1.setPosition(pos);
            left2.setPosition(pos);
            right1.setPosition(pos);
            right2.setPosition(pos);

        }

        lastPos = pos;
    }
}
