package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Config
@TeleOp(name = "Starter TeleOp")
public class StarterTeleOp extends LinearOpMode {

    MecanumPowerDrive drive;

    DcMotorEx slideMotor, pitchMotor;
    CRServo clawServo;

    //PIDFController slideController;

    public static double kP = 0, kI = 0, kD = 0, ff = 0;
    public static int target = 300;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        pitchMotor = hardwareMap.get(DcMotorEx.class, "pitchMotor");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int initialPitch = pitchMotor.getCurrentPosition();

        //clawServo.setPosition(0);

        //slideController = new PIDFController(kP, kI, kD, ff);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            boolean pitchDown = Math.abs((pitchMotor.getCurrentPosition() - initialPitch) - 900) < 20;
            boolean pitchMoving = Math.abs((pitchMotor.getCurrentPosition() - pitchMotor.getTargetPosition()) - 900) < 20;


            //slideController.setPIDF(kP, kI, kD, ff);

            int slidePos = slideMotor.getCurrentPosition();

            if (gamepad1.right_trigger > 0.05) {
                slideMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05) {
                slideMotor.setPower(-1 * gamepad1.left_trigger);
            } else {
                slideMotor.setPower(0);
            }

            drive.setRobotCentricPower(gamepad1.left_stick_x * 1.08,
                    -gamepad1.left_stick_y * 1.08, gamepad1.right_stick_x * 1.08);

//            if (gamepad1.triangle) {
//                target = 300;
//            }
//
//            if (gamepad1.circle) {
//                target = 150;
//            }
//
//            if (gamepad1.x) {
//                target = 10;
//            }

//            if (gamepad1.left_bumper) {
//
//                telemetry.addData("Pitch", "Moving");
//
//                if (pitchDown) {
//                    pitchMotor.setTargetPosition(950 + initialPitch); //85 degrees
//                } else {
//                    pitchMotor.setTargetPosition(900 + initialPitch); //(80/360) * 28 * 145
//                }
//
//            } else if (gamepad1.right_bumper) {
//
//                pitchMotor.setTargetPosition(initialPitch);
//            }
//
//            if (pitchMoving) {
//                pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

            if (gamepad1.left_bumper) {
                pitchMotor.setPower(1);
            } else if (gamepad1.right_bumper) {
                pitchMotor.setPower(-1);
            } else {
                pitchMotor.setPower(0);
            }


            //for Josh's claw
//            if (gamepad1.triangle) {
//                clawServo.setPosition(0.27);
//            } else if (gamepad1.circle) {
//                clawServo.setPosition(0);
//            }

            if (gamepad1.triangle) {
                clawServo.setPower(1);
            } else if (gamepad1.circle) {
                clawServo.setPower(-1);
            } else {
                clawServo.setPower(0);
            }

            telemetry.addData("Slide Target", target);
            telemetry.addData("Slide Power", slideMotor.getPower());
            telemetry.addData("Slide Position", slidePos);
            telemetry.addData("Pitch Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Target", pitchMotor.getTargetPosition());
            telemetry.addData("Pitch Down", pitchDown);
            telemetry.update();
        }
    }
}
