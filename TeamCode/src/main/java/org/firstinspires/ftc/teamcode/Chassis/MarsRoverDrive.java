package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MarsRoverDrive extends LinearOpMode {

    CRServo frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(CRServo.class, "FL");
        frontRight = hardwareMap.get(CRServo.class, "FR");
        backLeft = hardwareMap.get(CRServo.class, "BL");
        backRight = hardwareMap.get(CRServo.class, "BR");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.triangle) {
                frontLeft.setPower(1);
            } else if (gamepad1.dpad_up) {
                frontLeft.setPower(-1);
            }

            if (gamepad1.square) {
                backLeft.setPower(1);
            } else if (gamepad1.dpad_left) {
                backLeft.setPower(-1);
            }

            if (gamepad1.circle) {
                frontRight.setPower(1);
            } else if (gamepad1.dpad_right) {
                frontRight.setPower(-1);
            }

            if (gamepad1.x) {
                backRight.setPower(1);
            } else if (gamepad1.dpad_down) {
                backRight.setPower(-1);
            }

            telemetry.addData("Front Left", "Triangle/Up");
            telemetry.addData("Back Left", "Square/Left");
            telemetry.addData("Front Right", "Circle/Right");
            telemetry.addData("Back Right", "X/Down");

            telemetry.update();
        }

    }
}
