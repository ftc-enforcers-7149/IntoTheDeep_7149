package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TwoMotorSlides {

    private DcMotorEx leftMotor, rightMotor;

    public TwoMotorSlides(OpMode opMode) {
        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "frontSlidesLeft");
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "frontSlidesRight");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public int getPosition() {
        return rightMotor.getCurrentPosition();
    }

    public double getVelocity() {
        return rightMotor.getVelocity();
    }

    public void setMode(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }
}
