package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.SwerveModule;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.geometry.Translation2d;


@TeleOp(name = "ShanSwerveAttempt")
public class ShanSwerveAttempt extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SwerveModule fl = new SwerveModule(hardwareMap, "flWheel", "flPivot");
        SwerveModule fr = new SwerveModule(hardwareMap, "frWheel", "frPivot");
        SwerveModule bl = new SwerveModule(hardwareMap, "blWheel", "blPivot");
        SwerveModule br = new SwerveModule(hardwareMap, "brWheel", "brPivot");

        Translation2d m_frontLeftLocation =
                new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation =
                new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation =
                new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation =
                new Translation2d(-0.381, -0.381);

    }
}
