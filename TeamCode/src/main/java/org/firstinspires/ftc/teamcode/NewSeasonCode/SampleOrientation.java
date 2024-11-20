package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.VisionSubsystem;

@Disabled
@TeleOp(name = "Sample Orient")
public class SampleOrientation extends LinearOpMode {

    Servo servo;
    VisionSubsystem visionSystem;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.get(Servo.class, "servo");

        visionSystem = new VisionSubsystem(this);

        timer = new ElapsedTime();

        servo.setPosition(0.5);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            double servoPos = visionSystem.getDetector().getServoPos();

            if (timer.milliseconds() > 500) {
                servo.setPosition(servoPos);
                timer.reset();
            }

            telemetry.addData("Servo Pos", servoPos);
            telemetry.addData("Sample Angle", visionSystem.getDetector().sampleHead);
            telemetry.addData("World x", visionSystem.getDetector().contourX);
            telemetry.addData("World y", visionSystem.getDetector().contourY);

            FtcDashboard.getInstance().sendImage(visionSystem.getDetector().getLastFrame());

            telemetry.update();
        }
    }
}
