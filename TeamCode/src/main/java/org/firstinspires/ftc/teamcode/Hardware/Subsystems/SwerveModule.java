package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@Config
public class SwerveModule {
    public DcMotor driveMotor;
    public ServoImplEx pivotServo;

    public static final double PIVOT_RATIO = 1.6;
    public static final double SERVO_SWEEP = 288/355.0;

    public double lastServoPos = 0;
    public static final double VOLTAGE_TO_ANGLE = 360 / 3.3;

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName) {
        driveMotor = hardwareMap.get(DcMotor.class, motorName);
        pivotServo = hardwareMap.get(ServoImplEx.class, servoName);

        pivotServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setModuleState(double drivePower, double pivotPosition) {

        if (pivotPosition > 90) {

            pivotPosition -= 180;
            drivePower = -1;

        } else if (pivotPosition < -90) {

            pivotPosition += 180;
            drivePower= -1;
        }

        pivotPosition = (pivotPosition * PIVOT_RATIO) % 360;

        driveMotor.setPower(drivePower);

        pivotPosition = 0.5 + pivotPosition/355.0;

        pivotServo.setPosition(pivotPosition);
    }

    public void setModuleState(SwerveModuleState moduleState) {
        setModuleState(moduleState.speedMetersPerSecond, moduleState.angle.getDegrees());
    }

    public double getServoPosition() {
        return pivotServo.getPosition();
    }
}