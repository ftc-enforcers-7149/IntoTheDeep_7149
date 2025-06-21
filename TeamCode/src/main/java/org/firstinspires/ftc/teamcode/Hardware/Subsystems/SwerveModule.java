package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveModule {
    public DcMotor driveMotor;
    public Servo pivotServo;

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName) {
        driveMotor = hardwareMap.get(DcMotor.class, motorName);
        pivotServo = hardwareMap.get(Servo.class, servoName);
    }

    public void setModuleState(double drivePower, double pivotPosition) {
        driveMotor.setPower(drivePower);
        pivotServo.setPosition(pivotPosition);
    }
}
