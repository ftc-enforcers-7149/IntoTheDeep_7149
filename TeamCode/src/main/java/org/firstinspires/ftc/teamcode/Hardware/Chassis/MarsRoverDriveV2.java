package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp(name = "MarsRoverDriveV2")
public class MarsRoverDriveV2 extends LinearOpMode {

    public static double kP = 0.04, kI = .03, kD = 0, ff = 0;
    public static double pitchLimit = .05;
    public static double hinge1Pos = 144;
    public static double hinge2Pos = 65;
    public static double hinge3Pos = 310;
    PIDFController controller;
    PIDFController hinge1Control, hinge2Control, hinge3Control;

    public static double h1kP = 0.005, h1kI = 0.002, h1kD = 0, h1ff = 0;
    public static double h1step = 2, h2step = 5, h3step = -2;
    public static double h2kP = 0.005, h2kI = 0.002, h2kD = 0, h2ff = 0;
    public static double h3kP = 0.003, h3kI = 0.001, h3kD = 0, h3ff = 0;




    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo ldb = hardwareMap.get(CRServo.class, "ldb");
        CRServo ldm = hardwareMap.get(CRServo.class, "ldm");
        CRServo ldf = hardwareMap.get(CRServo.class, "ldf");
        CRServo rdb = hardwareMap.get(CRServo.class, "rdb");
        CRServo rdm = hardwareMap.get(CRServo.class, "rdm");
        CRServo rdf = hardwareMap.get(CRServo.class, "rdf");
        AnalogInput hinge1E = hardwareMap.get(AnalogInput.class, "hinge1Encoder");
        AnalogInput hinge2E = hardwareMap.get(AnalogInput.class, "hinge2Encoder");
        AnalogInput hinge3E = hardwareMap.get(AnalogInput.class, "hinge3Encoder");

        CRServo pitchm = hardwareMap.get(CRServo.class, "pitchm");

        ldb.setDirection(CRServo.Direction.REVERSE);
        ldm.setDirection(CRServo.Direction.REVERSE);
        ldf.setDirection(CRServo.Direction.REVERSE);

        Servo frontRightPivot = hardwareMap.get(Servo.class, "frSteer");
        Servo frontLeftPivot = hardwareMap.get(Servo.class, "flSteer");
        Servo backRightPivot = hardwareMap.get(Servo.class, "brSteer");
        Servo backLeftPivot = hardwareMap.get(Servo.class, "blSteer");
        Servo camPivot = hardwareMap.get(Servo.class, "camPiv");
//
        CRServo hinge1 = hardwareMap.get(CRServo.class, "hinge1");
        CRServo hinge2 = hardwareMap.get(CRServo.class, "hinge2");
        CRServo hinge3 = hardwareMap.get(CRServo.class, "hinge3");
        CRServo drill = hardwareMap.get(CRServo.class, "drill");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        controller = new PIDFController(kP, kI, kD, ff);
        hinge1Control = new PIDFController(h1kP, h1kI, h1kD, h1ff);
        hinge2Control = new PIDFController(h2kP, h2kI, h2kD, h2ff);
        hinge3Control = new PIDFController(h3kP, h3kI, h3kD, h3ff);

        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));

        imu.initialize(imuParams);

        final double frPos = .5 - .01;
        final double flPos = .5 + .04;
        final double brPos = .5 - .03;
        final double blPos = .5 + .01;

        frontRightPivot.setPosition(frPos);
        frontLeftPivot.setPosition(flPos);
        backRightPivot.setPosition(brPos);
        backLeftPivot.setPosition(blPos);

//        double hinge1Pos = hinge1.getPosition();
//        double hinge2Pos = hinge2.getPosition();
//        double hinge3Pos = hinge3.getPosition();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            double drivePower = -gamepad1.right_stick_y ;


            // 1 == left
            // 0 == right
            if (gamepad1.left_bumper) {
                frontRightPivot.setPosition(frPos + .16);
                frontLeftPivot.setPosition(flPos - .16);
                backLeftPivot.setPosition(blPos + .16);
                backRightPivot.setPosition(brPos - .16);
                sleep(200);
                rdb.setPower(1);
                rdm.setPower(1);
                rdf.setPower(1);
                ldb.setPower(-1);
                ldf.setPower(-1);
                ldm.setPower(-1);
            } else if (gamepad1.right_bumper) {
                frontRightPivot.setPosition(frPos + .16);
                frontLeftPivot.setPosition(flPos - .16);
                backLeftPivot.setPosition(blPos + .16);
                backRightPivot.setPosition(brPos - .16);
                sleep(200);
                rdb.setPower(-1);
                rdm.setPower(-1);
                rdf.setPower(-1);
                ldb.setPower(1);
                ldf.setPower(1);
                ldm.setPower(1);
            } else {
                frontRightPivot.setPosition(frPos);
                frontLeftPivot.setPosition(flPos);
                backLeftPivot.setPosition(blPos);
                backRightPivot.setPosition(brPos);

                rdb.setPower(drivePower);
                rdm.setPower(drivePower);
                rdf.setPower(drivePower);
                ldb.setPower(drivePower);
                ldf.setPower(drivePower);
                ldm.setPower(drivePower);
            }

//            if (gamepad1.dpad_left) {
//                hinge1Pos += h1step;  // move right, original .03
//            } else if (gamepad1.dpad_right) {
//                hinge1Pos -= h1step;  // move left
//            }
//
//            hinge1Pos = Math.max(87, Math.min(265, hinge1Pos));
//
//            hinge1.setPosition(hinge1Pos);
//            hinge1Control.setPIDF(h1kP, h1kI, h1kD, h1ff);
//            double h1angle = (hinge1E.getVoltage() / 3.3) * 360; // 87 - 265
//            double hinge1power = -hinge1Control.calculate(h1angle, hinge1Pos);
//
//            hinge1power = Math.max(-0.4, Math.min(.4, hinge1power));
//            hinge1.setPower(hinge1power);
//
//            double joystickY = gamepad1.left_stick_y;
//            if (Math.abs(joystickY) < 0.1) {
//                joystickY = 0;  // move right
//            } else {
//                joystickY = gamepad1.left_stick_y;
//            }
//
//            hinge2Pos += joystickY * 3.0;
//
//            hinge2Pos = Math.max(10, Math.min(140, hinge2Pos));
//            hinge2Control.setPIDF(h2kP, h2kI, h2kD, h2ff);
//            double h2angle = (hinge2E.getVoltage() / 3.3) * 360; // 87 - 265
//            double hinge2power = -hinge2Control.calculate(h2angle, hinge2Pos);
//            hinge2power = Math.max(-0.4, Math.min(.4, hinge2power));
//            hinge2.setPower(hinge2power);
//
//
//            if (gamepad1.dpad_up) {
//                hinge3Pos -= h3step;  // move right
//            } else if (gamepad1.dpad_down) {
//                hinge3Pos += h3step;  // move left
//            }
//
//            hinge3Control.setPIDF(h3kP, h3kI, h3kD, h3ff);
//            double h3angle = (hinge3E.getVoltage() / 3.3) * 360;
//
//            hinge3Pos = Math.max(7, Math.min(330, hinge3Pos));
//            double hinge3power = -hinge3Control.calculate(h3angle, hinge3Pos);
//            hinge3power = Math.max(-0.4, Math.min(.4, hinge3power));
//            hinge3.setPower(hinge3power);


            if (gamepad1.cross) {
                drill.setPower(1);
            } else {
                drill.setPower(0);
            }


            double pitch = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            double target = 0;
            double error = target - pitch;
            double pitchPower = 0;

            controller.setPIDF(kP, kI, kD, ff);

            pitchPower = controller.calculate(pitch, target);
            pitchPower = (Math.abs(pitchPower) < .1) ? 0 : pitchPower;
            pitchm.setPower(pitchPower);

            double camPos = .6;
            camPos -= gamepad1.left_trigger * .1;
            camPos += gamepad1.right_trigger * .1;

            camPos = Math.max(.3, Math.min(1, camPos));
            camPivot.setPosition(camPos);


            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Pitch Power", pitchPower);
//            telemetry.addData("Hinge1 Power", hinge1power);
//            telemetry.addData("Hinge1 Angle", hinge1Pos);
//            telemetry.addData("Hinge1 Location", h1angle);
//            telemetry.addData("Hinge2 Power", hinge2power);
//            telemetry.addData("Hinge2 Angle", hinge2Pos);
//            telemetry.addData("Hinge2 Location", h2angle);
//            telemetry.addData("Hinge3 Power", hinge3power);
//            telemetry.addData("Hinge3 Angle", hinge3Pos);
//            telemetry.addData("Hinge3 Location", h3angle);
            telemetry.update();
        }
    }

    public double[] invKin(double x, double y) {
        double l1length = 6.5;
        double l2length = 6.5;

        double D = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        double elbowAngle = Math.acos(D*D + l1length * l1length - l2length * l2length);

        double a1 = l1length * (x/D * Math.cos(elbowAngle) + y/D * Math.sin(elbowAngle));
        double b1 = l1length * (x/D * Math.sin(elbowAngle) + y/D * Math.cos(elbowAngle));

        double hinge2angle = Math.toDegrees(Math.atan2(a1, b1));
        double hinge3angle = Math.toDegrees(Math.atan2(x - a1, y - b1)) - hinge2angle;

        return new double[]{hinge2angle, hinge3angle};
    }
}