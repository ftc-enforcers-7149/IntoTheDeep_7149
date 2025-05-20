package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Subsystems.LLSampleVision;

@TeleOp(name = "Limelight Test", group = "Testers")
public class LimelightTester extends LinearOpMode {

    LLSampleVision vision;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        vision = new LLSampleVision(this, 0);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            double[] info = vision.getResultInfo();

            double orientation = info[0];

            orientation += Math.PI / 2;

            double servoPos = orientation / Math.PI;

            telemetry.addData("Info index 0", info[0]);
            telemetry.addData("Servo Pos", servoPos);
            telemetry.update();

        }

    }
}
