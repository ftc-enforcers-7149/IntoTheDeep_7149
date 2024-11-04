package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.VisionSubsystem;
import org.firstinspires.ftc.teamcode.NewSeasonCode.FSTTeleop_TwoClaws;

@TeleOp(name = "Sample Orient With Claw")
public class sampleOrientationWithClaw extends LinearOpMode {

    Servo servo;
    CRServo claw;
    VisionSubsystem visionSystem;

    ElapsedTime timer;

    Gamepad prevGamepad1, currentGamepad1;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.get(Servo.class, "servo");
        claw = hardwareMap.get(CRServo.class, "claw");

        visionSystem = new VisionSubsystem(this);

        timer = new ElapsedTime();

        servo.setPosition(0.5);

        prevGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();


        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            prevGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double servoPos = visionSystem.getDetector().getServoPos();

            if (timer.milliseconds() > 500) {
                servo.setPosition(servoPos);
                timer.reset();
            }

            //for claw intake and press again is stop
            if (gamepad1.left_bumper){
                claw.setPower(-1);
            }
            if (gamepad1.left_bumper && !(prevGamepad1.left_bumper)) {
                claw.setPower(0);
            }

            //for claw outtake and press again is stop
            if (gamepad1.right_bumper){
                claw.setPower(1);
            }
            if (gamepad1.right_bumper && !(prevGamepad1.right_bumper)) {
                claw.setPower(0);
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
