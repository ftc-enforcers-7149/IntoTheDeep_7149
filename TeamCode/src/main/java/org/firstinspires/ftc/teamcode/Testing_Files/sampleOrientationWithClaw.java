package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.VisionSubsystem;
import org.firstinspires.ftc.teamcode.NewSeasonCode.FSTTeleop_TwoClaws;

@Disabled
@Config
@TeleOp(name = "Sample Orient With Claw")
public class sampleOrientationWithClaw extends LinearOpMode {

    VisionSubsystem visionSystem;
    Servo clawWrist;

    OuttakeSlides slidesFront;
    OuttakeSlides slidesBack;
    CRServo clawFront;

    PIDFController pitchController;
    PIDFController slideController;
    PIDFController pitchController2;
    PIDFController slideController2;

    DcMotorEx pitchMotorFront;
    CRServo clawBack;
    DcMotorEx pitchMotorBack;

    MecanumPowerDrive drive;

    int slideFrontTarget = 0;
    int pitchFrontTarget = 0;
    double lastSlidePower = 0;
    double lastSlidePower2 = 0;

    int slideBackTarget = 0;
    int pitchBackTarget = 0;

    int slideBackInitPos;
    int pitchBackInitPos;

//    private enum Stages {
//        IDLE, INTAKING, OUTTAKING, SLIDEUP, SLIDEDOWN, PITCHUP, PITCHDOWN
//    }
//
//    private enum Outtake {
//        BUCKET, CHAMBER, HANG
//    }
//
//    private enum Intake {
//        ABOVE, PICKUP, IDLE, SPECIMEN_ABOVE, SPECIMEN_PICKUP
//    }
//
//    FSTTeleop_TwoClaws.Stages stageFront = FSTTeleop_TwoClaws.Stages.IDLE;
//    FSTTeleop_TwoClaws.Outtake outStageFront = FSTTeleop_TwoClaws.Outtake.BUCKET;
//    FSTTeleop_TwoClaws.Intake inStageFront = FSTTeleop_TwoClaws.Intake.IDLE;
//
//    FSTTeleop_TwoClaws.Stages stageBack = FSTTeleop_TwoClaws.Stages.IDLE;
//    FSTTeleop_TwoClaws.Outtake outStageBack = FSTTeleop_TwoClaws.Outtake.BUCKET;
//    FSTTeleop_TwoClaws.Intake inStageBack = FSTTeleop_TwoClaws.Intake.IDLE;

    Gamepad prevGamepad2, currentGamepad2;


    //Old sample above: 975, 1050
    public static int SAMPLE_ABOVE_PITCH_POS = 975, SAMPLE_PICKUP_PITCH_POS = 1050;
    final int SAMPLE_ABOVE_SLIDE_POS = 0, SAMPLE_PICKUP_SLIDE_POS = 0;
    final int SPECIMEN_ABOVE_PITCH_POS = 200, SPECIMEN_PICKUP_PITCH_POS = 400;

    final double backMotorOffsetAbove = 2.56;
    final double backMotorOffsetDown = 2.58;

    /*

    Right Trigger --> Pulls pitch down to inttake --> Pulls pitch down and turns on claw
    Left Trigger --> Pulls pitch up both stages, outtake at bucket

     */

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);

        visionSystem = new VisionSubsystem(this);

        slidesFront = new OuttakeSlides(hardwareMap, "frontSlide");

        slidesFront.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawFront = hardwareMap.get(CRServo.class, "frontClaw");
        clawWrist = hardwareMap.get(Servo.class, "frontWrist");
        pitchMotorFront = hardwareMap.get(DcMotorEx.class, "frontPitch");

        pitchMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);

        clawFront.setDirection(DcMotorSimple.Direction.REVERSE);

        pitchMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchController = new PIDFController(0.0102, 0, 0.0004,0);
        slideController = new PIDFController(0.008, 0, 0.00026, 0.00012);
        pitchController2 = new PIDFController(0.012, 0, 0.0003,0);
        slideController2 = new PIDFController(0.008, 0, 0.00026, 0.00012);

        slidesBack = new OuttakeSlides(hardwareMap, "backSlide");
        //slidesBack.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesBack.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawBack = hardwareMap.get(CRServo.class, "backClaw");
        pitchMotorBack = hardwareMap.get(DcMotorEx.class, "backPitch");
        pitchMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        slideBackInitPos = slidesBack.initialPos;

        prevGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        pitchMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesFront.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitchMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive() && !isStopRequested()) {

            //falling action gamepads
            prevGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            //field centric driving
            drive.setFieldCentricPower(gamepad1.left_stick_x * 1.08, -gamepad1.left_stick_y * 1.08, gamepad1.right_stick_x * 1.08);

            //FRONT PID POWERING
            double slidePower = slideController.calculate(slidesFront.slideMotor.getCurrentPosition(), slideFrontTarget);

            if (Math.abs(slidePower - lastSlidePower) > 0.02) {
                slidesFront.slideMotor.setPower(slidePower);
            }

            double pitchPower = pitchController.calculate(pitchMotorFront.getCurrentPosition(), pitchFrontTarget);
            //TODO
            //TODO
            pitchMotorFront.setPower(-gamepad1.right_stick_y);

            //BACK PID POWERING
            double slidePower2 = slideController2.calculate(slidesBack.slideMotor.getCurrentPosition(), slideBackTarget);

            if (Math.abs(slidePower2 - lastSlidePower2) > 0.02) {
                slidesBack.slideMotor.setPower(slidePower2);
            }

            double pitchPower2 = pitchController2.calculate(pitchMotorBack.getCurrentPosition(), pitchBackTarget);
            pitchMotorBack.setPower(pitchPower2);

            //reset heading for headless driving
            if (gamepad1.share) {
                drive.imu.resetYaw();
            }

            lastSlidePower = slidePower;
            lastSlidePower2 = slidePower2;

            //--------------------------------------

            double servoPos = visionSystem.getDetector().getServoPos();


            if (gamepad1.triangle) {
                clawWrist.setPosition(1 - servoPos);
            }


            if (gamepad1.left_bumper) {
                clawFront.setPower(-1);
            } else if (gamepad1.right_bumper) {
                clawFront.setPower(1);
            } else {
                clawFront.setPower(0);
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
