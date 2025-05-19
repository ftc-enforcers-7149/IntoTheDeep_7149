package org.firstinspires.ftc.teamcode.Hardware.Chassis;

 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MarsRoverFinal")
public class MarsRoverFinal extends LinearOpMode {

    // Drive motors (continuous rotation servos)
    CRServo leftDrive, rightDrive, drillServo;

    // Four independent steering servos
    Servo frontLeftPivot, frontRightPivot, backLeftPivot, backRightPivot;
    Servo pitchServo, cameraServo, armHinge1, armHinge2;

    // Inertial Measurement Unit
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        // Map hardware names to variables (must match RC config)
        leftDrive = hardwareMap.get(CRServo.class, "leftDrive");
        rightDrive = hardwareMap.get(CRServo.class, "rightDrive");
        drillServo = hardwareMap.get(CRServo.class, "drillServo");

        frontLeftPivot  = hardwareMap.get(Servo.class, "frontLeftPivot");
        frontRightPivot = hardwareMap.get(Servo.class, "frontRightPivot");
        backLeftPivot   = hardwareMap.get(Servo.class, "backLeftPivot");
        backRightPivot  = hardwareMap.get(Servo.class, "backRightPivot");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        armHinge1  = hardwareMap.get(Servo.class, "armHinge1");
        armHinge2 = hardwareMap.get(Servo.class, "armHinge2");

        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse one drive servo so both move the same way
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Center steering servos to forward position (0.5 = centered)
        frontLeftPivot.setPosition(0.5);
        frontRightPivot.setPosition(0.5);
        backLeftPivot.setPosition(0.5);
        backRightPivot.setPosition(0.5);

        // Set IMU orientation parameters
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        imu.initialize(imuParams);

        // Get pitch angle from IMU
        double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);

        // Map pitch angle to pitch servo (0 to 1 range)
        double pitchServoPos = (pitch + 90) / 180.0;
        pitchServoPos = Math.min(Math.max(pitchServoPos, 0), 1);
        pitchServo.setPosition(pitchServoPos);

        // Wait for the game to start
        waitForStart();

        if (isStopRequested()) return;

        // Main robot control loop
        while (opModeIsActive()) {

            // Set drive power from right joystick Y-axis
            double drivePower = -gamepad1.left_stick_y;

            // Steering control with left joystick X-axis
            double steerInput = gamepad1.right_stick_x;

            if (steerInput > 0.05) {
                // -1 = full left on x axis joystick
                // 1 = full right on x axis joystick
                // 1 = turn right, 0 = turn left
                // Right Crab walk
                frontLeftPivot.setPosition(1);
                frontRightPivot.setPosition(0);
                backLeftPivot.setPosition(0);
                backRightPivot.setPosition(1);
                leftDrive.setPower(1);
                rightDrive.setPower(-1);

            } else if (steerInput < -0.05) {
                // Left Crab Walk
                frontLeftPivot.setPosition(1);
                frontRightPivot.setPosition(0);
                backLeftPivot.setPosition(0);
                backRightPivot.setPosition(1);
                leftDrive.setPower(-1);
                rightDrive.setPower(1);

            } else {
                frontLeftPivot.setPosition(0.5);
                frontRightPivot.setPosition(0.5);
                backLeftPivot.setPosition(0.5);
                backRightPivot.setPosition(0.5);
                leftDrive.setPower(drivePower);
                rightDrive.setPower(drivePower);
            }

            double cameraPos = .5;

            if (gamepad1.left_bumper) {
                cameraServo.setPosition(cameraPos += .05);
            } else if (gamepad1.right_bumper) {
                cameraServo.setPosition(cameraPos += .05);
            }

            double armHinge1pos = .5;

            if (gamepad1.dpad_left) {
                armHinge1.setPosition(armHinge1pos - .05);
            } else if (gamepad1.dpad_right) {
                armHinge1.setPosition(armHinge1pos + .05);
            }

            double armHinge2pos = .5;

            if (gamepad1.dpad_up) {
                armHinge2.setPosition(armHinge2pos - .05);
            } else if (gamepad1.dpad_down) {
                armHinge2.setPosition(armHinge2pos + .05);
            }

            if (gamepad1.circle) {
                drillServo.setPower(1);
            } else {
                drillServo.setPower(.5);
            }

            // Telemetry for driver station
            telemetry.addData("Pitch (deg)", pitch);
            telemetry.addData("Pitch Servo Pos", pitchServoPos);
            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Steer Input", steerInput);

            telemetry.addData("Front Left Pivot", frontLeftPivot.getPosition());
            telemetry.addData("Front Right Pivot", frontRightPivot.getPosition());
            telemetry.addData("Back Left Pivot", backLeftPivot.getPosition());
            telemetry.addData("Back Right Pivot", backRightPivot.getPosition());

            telemetry.addData("Camera Servo Pos", cameraPos);

            telemetry.addData("Camera Servo Pos", cameraPos);
            telemetry.addData("ArmHinge1 Pos", armHinge1pos);
            telemetry.addData("ArmHinge2 Pos", armHinge2pos);

            telemetry.update();
        }
    }
}
