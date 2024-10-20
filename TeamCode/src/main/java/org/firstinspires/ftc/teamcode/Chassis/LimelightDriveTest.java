package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.text.DecimalFormat;

@TeleOp(name="LimelightDrive")
public class LimelightDriveTest extends LinearOpMode {

    MecanumPowerDrive drive;
    Limelight3A limelight;

    Pose2d pose;

    DecimalFormat format = new DecimalFormat("#.00");

    @Override
    public void runOpMode() throws InterruptedException {

        pose = new Pose2d(-48, 0,0);

        drive = new MecanumPowerDrive(hardwareMap, pose, telemetry);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        drive.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        drive.imu.resetYaw();

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            drive.setFieldCentricPower(gamepad1.left_stick_x * 1.08, -gamepad1.left_stick_y * 1.08, gamepad1.right_stick_x * 1.08);

            double heading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("Heading: ", heading);

            limelight.updateRobotOrientation(heading);

            LLResult llResult = limelight.getLatestResult();

            if (llResult != null) {
                if (llResult.isValid()) {
                    Pose3D botPose = llResult.getBotpose_MT2();
                    pose = new Pose2d(botPose.getPosition().x, botPose.getPosition().y, botPose.getOrientation().getYaw(AngleUnit.RADIANS));
                    telemetry.addData("Pos Updated", "");
                } else {
                    telemetry.addData("Invalid Result", "");
                }
            } else {
                telemetry.addData("Null Result", "");
            }

            telemetry.addData("Bot Pose", format.format(pose.position.x) + "  " + format.format( pose.position.y) + "  " + format.format(pose.heading.toDouble()));

            telemetry.update();

        }

    }
}
