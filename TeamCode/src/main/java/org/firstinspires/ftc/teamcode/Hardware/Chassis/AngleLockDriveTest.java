package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Config
@Autonomous(name = "AngleLockTest")
public class AngleLockDriveTest extends LinearOpMode {

    MecanumPowerDrive drive;

    boolean offAngle = false;
    boolean approachingTarget = false, prevApproachingTarget = false;

    public static double tolerance = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-48, 0, 0), telemetry);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            //update pose and get heading
            drive.updatePoseEstimate();
            double currentAngle = drive.imu.getRobotYawPitchRollAngles().getYaw();

            //determine if robot is off angle
            offAngle = Math.abs(currentAngle) < tolerance;

            //reset heading
            if (gamepad1.share) {
                drive.imu.resetYaw();
            }

            //start the recorrection process
            if (gamepad1.triangle && offAngle) {
                approachingTarget = true;
            } else if (!offAngle) {
                approachingTarget = false;
            }

            //if we just started approaching, reset the PID
            if (approachingTarget && !prevApproachingTarget) {
                drive.angPID.reset();
            }

            //if we are approaching and turning isn't interrupted,
            if ( approachingTarget && !(Math.abs(gamepad1.right_stick_x) < 0.01) ) {
                drive.setFieldCentricPower(gamepad1.left_stick_x * 1.08, -gamepad1.left_stick_y * 1.08,
                        drive.angPID.calculate(currentAngle, 0));
            } else {
                approachingTarget = false;
                drive.setFieldCentricPower(gamepad1.left_stick_x * 1.08, -gamepad1.left_stick_y * 1.08,
                        gamepad1.right_stick_x * 1.08);
            }

            prevApproachingTarget = approachingTarget;

        }

    }
}
