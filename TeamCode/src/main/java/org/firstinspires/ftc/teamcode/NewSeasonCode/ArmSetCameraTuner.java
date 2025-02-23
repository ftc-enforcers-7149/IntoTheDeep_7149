package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;

import java.util.ArrayList;

@TeleOp(name = "ArmSet Camera Tuner")
public class ArmSetCameraTuner extends LinearOpMode {

    //LLSampleVision llDetector;

    ElapsedTime wristTimer;
    ElapsedTime loopTime;

    ServoImplEx rightExt, leftExt;

    OuttakeSlides slidesFront;
    OuttakeSlides slidesBack;
    CRServo clawFront;
    Servo wristFront;
    Servo pitchBack1, pitchBack2;


    PIDFController pitchController;
    public static double kP = 0.026, kD = 0.00026;
    PIDFController slideController;
    PIDFController pitchController2;
    PIDFController slideController2;

    DcMotorEx pitchMotorFront;
    CRServo clawBack;
    //DcMotorEx pitchMotorBack;
    //DcMotorEx hangMotor;

    MecanumPowerDrive drive;

    int slideFrontTarget = 0;
    int pitchFrontTarget = 0;
    double lastSlidePower = 0;
    double lastSlidePower2 = 0;
    double lastPitchPower = 0;
    double lastPitchPower2 = 0;

    int pitchChange = 0;

    double frontWristPos = 0.5;
    double frontWristSpeed = 0.05;
    boolean changeWrist = false;

    double extensionPos = 1;

    final double extensionOutPos = 0.25;
    final double extensionInPos = 1;

    int slideBackTarget = 0;
    //    int pitchBackTarget = 0;
    double pitchBackpos = 0.0;


    int slideBackInitPos;
    int pitchBackInitPos;

    private enum Stages {
        IDLE, INTAKING, OUTTAKING, SLIDEUP, SLIDEDOWN, PITCHUP, PITCHDOWN
    }

    private enum Outtake {
        BUCKET, CHAMBER, HANG
    }

    private enum Intake {
        ABOVE, PICKUP, IDLE, SPECIMEN_ABOVE, SPECIMEN_PICKUP, SPECIMEN_OUT
    }

    ArmSetCameraTuner.Stages stageFront = ArmSetCameraTuner.Stages.IDLE;
    ArmSetCameraTuner.Outtake outStageFront = ArmSetCameraTuner.Outtake.BUCKET;
    ArmSetCameraTuner.Intake inStageFront = ArmSetCameraTuner.Intake.IDLE;

    ArmSetCameraTuner.Stages stageBack = ArmSetCameraTuner.Stages.IDLE;
    ArmSetCameraTuner.Outtake outStageBack = ArmSetCameraTuner.Outtake.BUCKET;
    ArmSetCameraTuner.Intake inStageBack = ArmSetCameraTuner.Intake.IDLE;

    Gamepad prevGamepad2, currentGamepad2;


    //Old sample above: 975, 1050

    //old w/o camera
//    final int SAMPLE_ABOVE_PITCH_POS = 910, SAMPLE_PICKUP_PITCH_POS = 1020;
    //    final int SAMPLE_ABOVE_PITCH_POS = 970, SAMPLE_PICKUP_P1ITCH_POS = 990;
    //with camera SAMPLE_PICKUP_PITCH_POS= 1005, 970
    public static int SAMPLE_ABOVE_PITCH_POS = 920, SAMPLE_PICKUP_PITCH_POS = 985, SAMPLE_OUT_PITCH_POS = 950;
    //new with camera
    final int SAMPLE_ABOVE_SLIDE_POS = 100, SAMPLE_PICKUP_SLIDE_POS = 100;
    final int SPECIMEN_ABOVE_PITCH_POS = 200, SPECIMEN_PICKUP_PITCH_POS = 400, SPECIMEN_OUT_PITCH_POS = 400;
    final int FRONT_SLIDE_HIGH_BASKET = 1980, FRONT_SLIDE_LOW_BASKET = 600, BACK_SLIDE_HIGH_CHAMBER = 610, BACK_SLIDE_LOW_CHAMBER = 50; //TODO high & low basket values
    final double BACK_PITCH_IDLE = 0.0, BACK_PITCH_ABOVE = 0.36, BACK_PITCH_PICKUP = 0.4, BACK_PITCH_WALL = 0.133, BACK_PITCH_CHAMBER = 0.111;
    //final double BACK_PITCH_IDLE = 0.4, BACK_PITCH_ABOVE = 0.04, BACK_PITCH_PICKUP = 0.0, BACK_PITCH_WALL = 0.267, BACK_PITCH_CHAMBER = 0.289; for when we reverse the servos and stuff so we can hang
//NEW ANGLES 25 degrees = .111, 30 degrees = .1333, 35 degrees = .1554, 40 degrees = .1778, 45 degrees = .2
// 1 degree = .00444444444444

    /*

    Right Trigger --> Pulls pitch down to inttake --> Pulls pitch down and turns on claw
    Left Trigger --> Pulls pitch up both stages, outtake at bucket

     */

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0, 0, 0), telemetry);

        slidesFront = new OuttakeSlides(hardwareMap, "frontSlide");
        slidesFront.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //right side
        rightExt = hardwareMap.get(ServoImplEx.class, "rightExt");
        rightExt.setPwmRange(new PwmControl.PwmRange(500, 2500));
        leftExt = hardwareMap.get(ServoImplEx.class, "leftExt");
        leftExt.setPwmRange(new PwmControl.PwmRange(500, 2500));

        loopTime = new ElapsedTime();

        //slidesFront.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawFront = hardwareMap.get(CRServo.class, "frontClaw");
        wristFront = hardwareMap.get(Servo.class, "frontWrist");
        wristTimer = new ElapsedTime();

        pitchMotorFront = hardwareMap.get(DcMotorEx.class, "frontPitch");

        pitchMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);

        clawFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //pitchMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchController = new PIDFController(0.022, 0, 0.00065, 0);
        //good
        slideController = new PIDFController(0.025, 0, 0.0004, 0.00016);
        //good
        pitchController2 = new PIDFController(0.012, 0, 0.0003, 0);
        //good
        slideController2 = new PIDFController(0.02, 0, 0.0004, 0.0002);
        //good
        slidesBack = new OuttakeSlides(hardwareMap, "backSlide");
        slidesBack.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //slidesBack.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        clawBack = hardwareMap.get(CRServo.class, "backClaw");


        pitchBack1 = hardwareMap.get(Servo.class, "pitchBack1");
        pitchBack2 = hardwareMap.get(Servo.class, "pitchBack2");

        //hang motor
        //   hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");


        //pitchMotorBack = hardwareMap.get(DcMotorEx.class, "backPitch");
//change

        //pitchMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        slideBackInitPos = slidesBack.initialPos;

        prevGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();

        //llDetector = new LLSampleVision(this, 0);

        ArrayList<LynxModule> allHubs = new ArrayList<LynxModule>(hardwareMap.getAll(LynxModule.class));
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        pitchMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesFront.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //pitchMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.options) {
                pitchController.setPIDF(kP, 0, kD, 0);
            }

            //reset stale readings for bulk reads
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //falling action gamepads
            prevGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            //field centric driving
            drive.setFieldCentricPower(gamepad1.left_stick_x * 1.08, -gamepad1.left_stick_y * 1.08, gamepad1.right_stick_x * 1.08);

            //FRONT PID POWERING
            double slidePower = slideController.calculate(slidesFront.slideMotor.getCurrentPosition(), slideFrontTarget);

            //if (Math.abs(slidePower - lastSlidePower) > 0.02) {
            slidesFront.slideMotor.setPower(slidePower);
            //}

            double pitchPower = pitchController.calculate(pitchMotorFront.getCurrentPosition(), pitchFrontTarget);
            pitchMotorFront.setPower(pitchPower);


            //hang motor
            // hangMotor.setPower(gamepad2.right_stick_y);

            //BACK PID POWERING
            double slidePower2 = slideController2.calculate(slidesBack.slideMotor.getCurrentPosition(), slideBackTarget);

            if (Math.abs(slidePower2 - lastSlidePower2) > 0.02) {
                slidesBack.slideMotor.setPower(slidePower2);
            }

//            double pitchPower2 = pitchController2.calculate(pitchMotorBack.getCurrentPosition(), pitchBackTarget);
//            if (Math.abs(pitchPower2 - lastPitchPower2) > 0.02) {
//                pitchMotorBack.setPower(pitchPower2);
//            }

            wristFront.setPosition(1 - frontWristPos);

            pitchBack1.setPosition(pitchBackpos);
            pitchBack2.setPosition(pitchBackpos);


            rightExt.setPosition(extensionPos);
            leftExt.setPosition(extensionPos);

            //reset heading for headless driving
            if (gamepad1.share) {
                drive.imu.resetYaw();
            }

            lastSlidePower = slidePower;
            lastSlidePower2 = slidePower2;
            lastPitchPower = pitchPower;


            //=============== CODE GOES HERE ======================

            pitchFrontTarget = 400;
            extensionPos = extensionOutPos;

        }

    }
}
