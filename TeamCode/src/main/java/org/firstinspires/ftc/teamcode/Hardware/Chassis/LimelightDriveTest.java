package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.text.DecimalFormat;
import java.util.List;

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
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
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

            double heading = MecanumPowerDrive.AngleWrap(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.toRadians(90));

            if (gamepad1.share) {
                drive.imu.resetYaw();
            }

            telemetry.addData("Heading: ", heading);
            telemetry.addData("", "");

            limelight.updateRobotOrientation(heading);

            LLResult llResult = limelight.getLatestResult();

            if (llResult != null) {
                if (llResult.isValid()) {

                    List<LLResultTypes.FiducialResult> aprilTags = llResult.getFiducialResults();

                    for (LLResultTypes.FiducialResult tag : aprilTags) {
                        double x = tag.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x;
                        double y = tag.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y;
                        telemetry.addData("Tag ID", tag.getFiducialId());
                        telemetry.addData("Tag Pose", x + "  " + y);
                    }
//
//                    telemetry.addData("Tx", llResult.getTx());
//                    telemetry.addData("Ty", llResult.getTy());

                    Pose3D botPose = llResult.getBotpose_MT2();
                    Pose3D botPose1 = llResult.getBotpose();


                    pose = new Pose2d(-1 * botPose.getPosition().toUnit(DistanceUnit.INCH).y, botPose.getPosition().toUnit(DistanceUnit.INCH).x, heading);

                    telemetry.addData("MT2 Pose", botPose.getPosition().toUnit(DistanceUnit.INCH).x + "  " + botPose.getPosition().toUnit(DistanceUnit.INCH).y + "  " + botPose.getOrientation().getYaw(AngleUnit.RADIANS) );
                    telemetry.addData("MT1 Pose", botPose1.getPosition().toUnit(DistanceUnit.INCH).x + "  " + botPose1.getPosition().toUnit(DistanceUnit.INCH).y + "  " + botPose1.getOrientation().getYaw(AngleUnit.RADIANS) );

                    telemetry.addData("Pos Updated", "");


                } else {
                    telemetry.addData("Invalid Result", "");
                }
            } else {
                telemetry.addData("Null Result", "");
            }

            telemetry.addData("Bot Pose (x,y)", format.format(pose.position.x) + "  " + format.format(pose.position.y) );
            telemetry.update();

        }

    }
}