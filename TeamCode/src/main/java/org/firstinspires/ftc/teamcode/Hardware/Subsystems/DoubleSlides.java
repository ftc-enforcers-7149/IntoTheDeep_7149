package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

public class DoubleSlides implements PeriodicAction {

    //as viewed from the front of the robot
    private DcMotorEx leftMotor, rightMotor;
    private PIDFController controller;
    private int target;

    public DoubleSlides(OpMode opMode) {
        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "leftSlides");
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "rightSlides");

        controller = new PIDFController(0,0,0,0);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int getCurrentPosition() {
        int leftPos = leftMotor.getCurrentPosition();
        int rightPos = rightMotor.getCurrentPosition();

        //if an encoder gets unplugged, use the other
        if (leftPos == 0 && rightPos != 0) {
            return rightPos;
        } else if (rightPos == 0 && leftPos != 0) {
            return leftPos;
        } else {
            return (leftPos + rightPos) / 2;
        }
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    @Override
    public void periodic() {

        double slidePow = controller.calculate(getCurrentPosition(), target);
        setPower(slidePow);

    }
}
