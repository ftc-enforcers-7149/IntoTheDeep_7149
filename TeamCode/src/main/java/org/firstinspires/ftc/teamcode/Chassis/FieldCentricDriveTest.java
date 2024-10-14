package org.firstinspires.ftc.teamcode.Chassis;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name  ="FieldCentric-Drive", group = "Chassis")
public class FieldCentricDriveTest extends LinearOpMode{

    DcMotorEx frontLeft, backLeft, frontRight, backRight;

    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

//        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(
//                AxesReference.INTRINSIC,
//                AxesOrder.ZYX,
//                AngleUnit.DEGREES,
//                -90,
//                0,
//                0,
//                0
//        )));

        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(imuParams);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -1 * gamepad1.left_stick_y * 1.08;
            double x = gamepad1.left_stick_x * 1.08 * 1.1;
            double rx = gamepad1.right_stick_x * 1.08;

            //joystick values have to be counterrotated by the heading to maintain the same heading,
            //therefore the negative heading is applied to rotate in opposite direction
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("FL" ,frontLeftPower);
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Heading (rad)", heading);
            telemetry.update();
        }
    }
}
