package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MarsRoverV1")
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


            frontLeft.setPower(-gamepad1.left_stick_y);
            backLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.right_stick_y);
            backRight.setPower(-gamepad1.right_stick_y);



            telemetry.addData("Front Left", "Triangle/Up");
            telemetry.addData("Back Left", "Square/Left");
            telemetry.addData("Front Right", "Circle/Right");
            telemetry.addData("Back Right", "X/Down");

            telemetry.addData("BL Power", backLeft.getPower());
            telemetry.addData("FL Power", frontLeft.getPower());
            telemetry.addData("BR Power", backRight.getPower());
            telemetry.addData("FR Power", frontRight.getPower());

            telemetry.update();
        }

    }
}
