package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MarsRover TeleOp")
public class MarsRoverRev extends LinearOpMode {

    CRServo leftDrive, rightDrive;
    Servo leftPivot, rightPivot, chassisPivot;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(CRServo.class, "leftDrive");
        rightDrive = hardwareMap.get(CRServo.class, "rightDrive");

        leftPivot = hardwareMap.get(Servo.class, "frontSteer");
        rightPivot = hardwareMap.get(Servo.class, "backSteer");
        chassisPivot = hardwareMap.get(Servo.class, "pitchServo");

        leftPivot.setPosition(0.5);
        rightPivot.setPosition(0.5);

        imu = hardwareMap.get(IMU.class, "imu");

        //fakes the position of the imu so that the pitch is on the yaw axis
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        imu.initialize(params);
        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            double pitch = imu.getRobotYawPitchRollAngles().getPitch() + 30;
            telemetry.addData("Pitch", pitch);

            double pitchPos = (pitch + 90) / 180.0;
            chassisPivot.setPosition(pitchPos);

            leftDrive.setPower(-gamepad1.right_stick_y);
            rightDrive.setPower(gamepad1.right_stick_y);

            if (gamepad1.left_stick_x > 0.05) {
                rightPivot.setPosition(1);
                leftPivot.setPosition(1);
            } else if (gamepad1.left_stick_x < -0.05) {
                rightPivot.setPosition(0);
                leftPivot.setPosition(0);
            } else {
                leftPivot.setPosition(0.5);
                rightPivot.setPosition(0.5);
            }


            telemetry.update();
        }
    }
}
