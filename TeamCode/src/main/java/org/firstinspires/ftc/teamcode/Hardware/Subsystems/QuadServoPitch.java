package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

public class QuadServoPitch implements PeriodicAction {

    //as viewed from the front of the robot
    private CRServo leftServoBottom, leftServoTop, rightServoBottom, rightServoTop;
    private AxonAbsoluteEncoder pitchEncoder;
    private PIDFController controller;

    private double pitchPos, target, pitchAngle;
    private final double POS_PITCH_CONVERSION = 1/3.0;

    private final double ff = 0.03;

    public QuadServoPitch(OpMode opMode) {
        leftServoBottom = opMode.hardwareMap.get(CRServo.class, "leftBottom");
        leftServoTop = opMode.hardwareMap.get(CRServo.class, "leftTop");
        rightServoBottom = opMode.hardwareMap.get(CRServo.class, "rightBottom");
        rightServoTop = opMode.hardwareMap.get(CRServo.class, "rightTop");

        controller = new PIDFController(0,0,0,0);

        pitchEncoder = new AxonAbsoluteEncoder(opMode.hardwareMap, "pitchInput");

        pitchPos = pitchEncoder.getAbsolutePosition();
    }

    public void setPower(double power) {
        leftServoBottom.setPower(power);
        leftServoTop.setPower(power);
        rightServoBottom.setPower(power);
        rightServoTop.setPower(power);
    }

    /**
     * @return The position of the pitch from wraparound encoder
     */
    public double getPosition() {
        return pitchPos;
    }

    public double getPitchAngle() {
        return pitchAngle;
    }

    @Override
    public void periodic() {

        //update the encoder inside this periodic call
        pitchEncoder.periodic();

        //get position from encoder and convert to an angle measure
        pitchPos = pitchEncoder.getAbsolutePosition();
        pitchAngle = pitchPos * POS_PITCH_CONVERSION;

        //adds angle based feedforward
        double pitchPow = controller.calculate(pitchPos, target) + (Math.cos(Math.toRadians(90 - pitchAngle)) * ff);

        setPower(pitchPow);
    }
}
