package org.firstinspires.ftc.teamcode.Testing_Files.V3Testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.ClawDifferential;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.DoubleSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.OuttakeExtendo;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.QuadServoPitch;

public class V3Tester extends LinearOpMode {

    ClawDifferential diffyClaw;
    OuttakeExtendo extendo;
    QuadServoPitch pitch;
    DoubleSlides slides;
    MecanumPowerDrive drive;

    public static int slidePos = 0;
    public static double diffyPitch = 0, diffyRotation = 0, pitchPos = 0, extendoPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);
        drive.initializeIMU(hardwareMap, new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        diffyClaw = new ClawDifferential(this);
        extendo = new OuttakeExtendo(this);
        pitch = new QuadServoPitch(this);
        slides = new DoubleSlides(this);

        diffyClaw.initPositions();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            drive.setRobotCentricPower(gamepad1.left_stick_x * 1.08,
                    -1 * gamepad1.left_stick_y * 1.08,
                    gamepad1.right_stick_x * 1.08);

            pitch.setPower(gamepad1.left_stick_x);
            pitch.updateEncoder();

            diffyPitch += gamepad1.right_stick_y * 2;
            diffyRotation += gamepad1.right_stick_x * 2;

            diffyClaw.setPitchRotAngles(diffyPitch, diffyRotation);

            if (gamepad1.right_bumper) {
                extendoPos += 0.01;
            } else if (gamepad1.left_bumper) {
                extendoPos -= 0.01;
            }

            extendo.setExtension(extendoPos);

            telemetry.addData("Pitch Pos", pitch.getPosition());
            telemetry.addData("Diffy L", diffyClaw.getLeftPos());
            telemetry.addData("Diffy R", diffyClaw.getRightPos());
            telemetry.addData("Diffy Pitch", diffyClaw.getPitchAngle());
            telemetry.addData("Diffy Rotation", diffyClaw.getRotationAngle());
            telemetry.addData("Extendo Pos", extendo.getExtensionPos());

            telemetry.update();
        }

    }
}
