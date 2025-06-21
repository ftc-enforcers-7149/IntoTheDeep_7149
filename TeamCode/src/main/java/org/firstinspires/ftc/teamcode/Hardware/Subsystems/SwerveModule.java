package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class SwerveModule {
    public DcMotor driveMotor;
    public CRServo pivotServo;
    public AnalogInput axonEncoder;
    public PIDFController crController;
    public static double kP, kD;

    public static final double VOLTAGE_TO_ANGLE = 360 / 3.3;

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName, String encoderName) {
        driveMotor = hardwareMap.get(DcMotor.class, motorName);
        pivotServo = hardwareMap.get(CRServo.class, servoName);
        axonEncoder = hardwareMap.get(AnalogInput.class, encoderName);

        crController = new PIDFController(kP, 0, kD, 0);
    }

    public void setModuleState(double drivePower, double pivotPosition) {
        driveMotor.setPower(drivePower);

        double power = crController.calculate(axonEncoder.getVoltage() * VOLTAGE_TO_ANGLE, pivotPosition);
        pivotServo.setPower(power);
    }

    public void setModuleState(SwerveModuleState moduleState) {
        setModuleState(moduleState.speedMetersPerSecond, moduleState.angle.getDegrees());
    }
}
