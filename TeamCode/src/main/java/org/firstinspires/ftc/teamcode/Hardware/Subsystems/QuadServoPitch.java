package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class QuadServoPitch {

    //as viewed from the front of the robot
    private CRServo leftServoBottom, leftServoTop, rightServoBottom, rightServoTop;
    private AnalogInput pitchPositionInput;

    private double pitchPos;

    public QuadServoPitch(OpMode opMode) {
        leftServoBottom = opMode.hardwareMap.get(CRServo.class, "leftBottom");
        leftServoTop = opMode.hardwareMap.get(CRServo.class, "leftTop");
        rightServoBottom = opMode.hardwareMap.get(CRServo.class, "rightBottom");
        rightServoTop = opMode.hardwareMap.get(CRServo.class, "rightTop");

        pitchPositionInput = opMode.hardwareMap.get(AnalogInput.class, "pitchInput");

        pitchPos = 0;
    }

    public void setPower(double power) {
        leftServoBottom.setPower(power);
        leftServoTop.setPower(power);
        rightServoBottom.setPower(power);
        rightServoTop.setPower(power);
    }

    /**
     * @return The position of the pitch in degrees (0, 360)
     */
    public double getPosition() {
        pitchPos = pitchPositionInput.getVoltage() / 3.3 * 360;
        return pitchPos;
    }
}
