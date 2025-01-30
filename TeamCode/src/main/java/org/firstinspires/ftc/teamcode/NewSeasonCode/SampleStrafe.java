package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.VisionSubsystem;
import org.opencv.core.RotatedRect;

@Disabled
@Config
@TeleOp(name = "SampleStrafe")
public class SampleStrafe extends LinearOpMode {

    MecanumPowerDrive drive;
    VisionSubsystem visionSystem;

    PIDFController strafeController;

    public static double p, i, d, f;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-48, 0, 0), telemetry);
        visionSystem = new VisionSubsystem(this);

        strafeController = new PIDFController(p, i, d, f);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            strafeController.setPIDF(p, i, d, f);

            RotatedRect bestSample = visionSystem.getDetector().getBestSample();

            double xStrafeOffset = bestSample.center.x - 160;
            double strafePower = strafeController.calculate(xStrafeOffset, 0);

            drive.setRobotCentricPower(strafePower, 0, 0);

            telemetry.addData("xStrafeOffset", xStrafeOffset);
            telemetry.addData("Target", 0);
            telemetry.addData("strafePower", strafePower);
            telemetry.update();

        }
    }
}
