package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.SwerveModule;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.geometry.Translation2d;

@TeleOp(name = "ShanSwerveAttempt")
public class ShanSwerveAttempt extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SwerveModule fl = new SwerveModule(hardwareMap, "flWheel", "flPivot", "flEncoder");
        SwerveModule fr = new SwerveModule(hardwareMap, "frWheel", "frPivot", "frEncoder");
        SwerveModule bl = new SwerveModule(hardwareMap, "blWheel", "blPivot", "blEncoder");
        SwerveModule br = new SwerveModule(hardwareMap, "brWheel", "brPivot", "brEncoder");

        SwerveModule[] swerveModules = {fl, fr, bl, br};

        Translation2d m_frontLeftLocation =
                new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation =
                new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation =
                new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation =
                new Translation2d(-0.381, -0.381);

        // Creating my kinematics object using the module locations
        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotation);

            SwerveModuleState[] moduleStates =
                    m_kinematics.toSwerveModuleStates(speeds);

            for (int i = 0; i < 4; i++) {
                swerveModules[i].setModuleState(moduleStates[i]);
            }

        }
    }
}
