package org.firstinspires.ftc.teamcode.PathingSystems.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.PathingSystems.RRTuning.Drawing;

@Config
@TeleOp(name = "P2P Tuner")
public class P2PTuner extends LinearOpMode {

    ElapsedTime timer;

    public static double X = 0, Y = 0, ANG = 90;

    public static double moveSpeed = 5, turnSpeed = 5;

    MecanumPowerDrive drive;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {

        double initialAng = ANG;

        timer = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(X,Y,Math.toRadians(ANG)), telemetry);
        drive.imu.resetYaw();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while(opModeIsActive() && !isStopRequested()) {
            drive.updatePoseEstimate();

            if (gamepad1.triangle) {
                drive.xPID.reset();
                drive.yPID.reset();
                drive.angPID.reset();
                telemetry.addData("Reset", "PID");
                telemetry.update();
            }

            TelemetryPacket p2pPacket = new TelemetryPacket();
            Canvas c = p2pPacket.fieldOverlay();

            if (timer.milliseconds() > 250) {
                drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(initialAng));
                timer.reset();
            }


            drive.goToPose(X, Y, Math.toRadians(ANG), moveSpeed, turnSpeed);

            //================Displaying info on FTC Dashboard Field=============
            Drawing.drawRobot(c, drive.pose);
            c.setFill("green");
            c.fillCircle(X, Y, 2);
            c.setStroke("green");
            c.strokeLine(X, Y, 5 * Math.cos(Math.toRadians(ANG)) + X, 5 * Math.sin(Math.toRadians(ANG)) + Y);

            dashboard.sendTelemetryPacket(p2pPacket);
            //====================================================================


            telemetry.addData("xPos", drive.pose.position.x);
            telemetry.addData("yPos", drive.pose.position.y);
            telemetry.addData("angPos(RAD)", drive.pose.heading.toDouble());
            telemetry.addData("angPos(DEG)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("xTarget", X);
            telemetry.addData("yTarget", Y);
            telemetry.addData("angTarget", Math.toRadians(ANG));
            telemetry.update();
        }


    }
}
