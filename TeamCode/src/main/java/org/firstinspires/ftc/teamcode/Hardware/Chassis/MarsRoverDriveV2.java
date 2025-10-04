package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MarsRoverDriveV2")
public class MarsRoverDriveV2 extends LinearOpMode {

    double kP = 0.02;
    double kI = 0.0;
    double kD = 0.001;

    double integral = 0;
    double lastError = 0;
    double lastTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo leftDrive = hardwareMap.get(CRServo.class, "lDrive");
        CRServo rightDrive = hardwareMap.get(CRServo.class, "rDrive");
        CRServo leftDriveb = hardwareMap.get(CRServo.class, "lDriveb");
        CRServo pitchServo = hardwareMap.get(CRServo.class, "pitchServo");

        leftDrive.setDirection(CRServo.Direction.REVERSE);
        leftDriveb.setDirection(CRServo.Direction.REVERSE);
        pitchServo.setDirection(CRServo.Direction.REVERSE);

        Servo frontRightPivot = hardwareMap.get(Servo.class, "frSteer");
        Servo frontLeftPivot = hardwareMap.get(Servo.class, "flSteer");
        Servo backRightPivot = hardwareMap.get(Servo.class, "brSteer");
        Servo backLeftPivot = hardwareMap.get(Servo.class, "blSteer");

        Servo hinge1 = hardwareMap.get(Servo.class, "hinge1");
        Servo hinge2 = hardwareMap.get(Servo.class, "hinge2");
        Servo hinge3 = hardwareMap.get(Servo.class, "hinge3");
        CRServo drill = hardwareMap.get(CRServo.class, "drill");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));

        imu.initialize(imuParams);

        frontRightPivot.setPosition(0.5);
        frontLeftPivot.setPosition(0.5);
        backRightPivot.setPosition(0.5);
        backLeftPivot.setPosition(0.5);

        double hinge1Pos = .5;
        double hinge2Pos = .5;
        double hinge3Pos = .5;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            double drivePower = -gamepad1.right_stick_y  ;
            // 1 == left
            // 0 == right
            if (gamepad1.left_bumper) {
                frontRightPivot.setPosition(1);
                frontLeftPivot.setPosition(0);
                backLeftPivot.setPosition(1);
                backRightPivot.setPosition(0);

                rightDrive.setPower(1);
                leftDrive.setPower(-1);
                leftDriveb.setPower(-1);
            } else if (gamepad1.right_bumper) {
                frontRightPivot.setPosition(1);
                frontLeftPivot.setPosition(0);
                backLeftPivot.setPosition(1);
                backRightPivot.setPosition(0);

                rightDrive.setPower(-1);
                leftDrive.setPower(1);
                leftDriveb.setPower(1);
            } else {
                frontRightPivot.setPosition(0.5);
                frontLeftPivot.setPosition(0.5);
                backLeftPivot.setPosition(0.5);
                backRightPivot.setPosition(0.5);

                rightDrive.setPower(drivePower);
                leftDrive.setPower(drivePower);
                leftDriveb.setPower(drivePower);
            }

            if (gamepad1.dpad_right) {
                hinge1Pos += 0.03;  // move right
                sleep(50);
            } else if (gamepad1.dpad_left) {
                hinge1Pos -= 0.03;  // move left
                sleep(50);
            }

            hinge1Pos = Math.max(0, Math.min(1, hinge1Pos));

            hinge1.setPosition(hinge1Pos);

            if (-gamepad1.left_stick_y > 0.1) {
                hinge2Pos -= 0.03;  // move right
                sleep(50);
            } else if (-gamepad1.left_stick_y < -0.1) {
                hinge2Pos += 0.03;  // move left
                sleep(50);
            }

            hinge2Pos = Math.max(0, Math.min(1, hinge2Pos));

            hinge2.setPosition(hinge2Pos);

            if (gamepad1.dpad_up) {
                hinge3Pos -= 0.03;  // move right
                sleep(50);
            } else if (gamepad1.dpad_down) {
                hinge3Pos += 0.03;  // move left
                sleep(50);
            } else {
                hinge3Pos += 0;
            }

            hinge3Pos = Math.max(0, Math.min(1, hinge3Pos));

            hinge3.setPosition(hinge3Pos);

            if (gamepad1.a) {
                while(gamepad1.a) {
                    drill.setPower(1);
                    drill.setPower(0);
                }
            } else {
                drill.setPower(0);
            }

            double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            double targetPitch = 0; // keep parallel to ground

            // PID error
            double error = targetPitch - pitch;
            double now = System.nanoTime();
            double dt = (now - lastTime) / 1e9 ; // seconds

            integral += error * dt;
            double derivative = (error - lastError) / dt;

            double output = (kP * error) + (kI * integral) + (kD * derivative);

            // Clamp to CRServo power
            output = Math.max(-1, Math.min(output, 1));

            pitchServo.setPower(output);

            lastError = error;
            lastTime = now;

            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Hinge1", hinge1Pos);
            telemetry.addData("Hinge2", hinge2Pos);
            telemetry.addData("Hinge3", hinge3Pos);
            telemetry.update();
        }
    }
}
