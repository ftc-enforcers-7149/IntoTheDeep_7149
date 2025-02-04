package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;

import java.util.ArrayList;

@Config
@TeleOp(name = "FST TeleOp Main")
public class FSTTeleop_TwoClaws extends LinearOpMode {

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
    DcMotorEx hangMotor;

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

    Stages stageFront = Stages.IDLE;
    Outtake outStageFront = Outtake.BUCKET;
    Intake inStageFront = Intake.IDLE;

    Stages stageBack = Stages.IDLE;
    Outtake outStageBack = Outtake.BUCKET;
    Intake inStageBack = Intake.IDLE;

    Gamepad prevGamepad2, currentGamepad2;


    //Old sample above: 975, 1050

    //old w/o camera
//    final int SAMPLE_ABOVE_PITCH_POS = 910, SAMPLE_PICKUP_PITCH_POS = 1020;
    //    final int SAMPLE_ABOVE_PITCH_POS = 970, SAMPLE_PICKUP_P1ITCH_POS = 990;
    //with camera SAMPLE_PICKUP_PITCH_POS= 1005, 970
    public static int SAMPLE_ABOVE_PITCH_POS = 920, SAMPLE_PICKUP_PITCH_POS = 1000, SAMPLE_OUT_PITCH_POS = 960;
    //new with camera
    final int SAMPLE_ABOVE_SLIDE_POS = 100, SAMPLE_PICKUP_SLIDE_POS = 100;
    final int SPECIMEN_ABOVE_PITCH_POS = 200, SPECIMEN_PICKUP_PITCH_POS = 400, SPECIMEN_OUT_PITCH_POS = 400;
    final int FRONT_SLIDE_HIGH_BASKET = 0, FRONT_SLIDE_LOW_BASKET = 0, BACK_SLIDE_HIGH_CHAMBER = 625, BACK_SLIDE_LOW_CHAMBER = 50;
    final double BACK_PITCH_IDLE = 0.0, BACK_PITCH_ABOVE = 0.36, BACK_PITCH_PICKUP = 0.4, BACK_PITCH_WALL = 0.133, BACK_PITCH_CHAMBER = 0.111;
    //final double BACK_PITCH_IDLE = 0.4, BACK_PITCH_ABOVE = 0.04, BACK_PITCH_PICKUP = 0.0, BACK_PITCH_WALL = 0.267, BACK_PITCH_CHAMBER = 0.289;
//NEW ANGLES 25 degrees = .111, 30 degrees = .1333, 35 degrees = .1554, 40 degrees = .1778, 45 degrees = .2
// 1 degree = .00444444444444

    /*

    Right Trigger --> Pulls pitch down to inttake --> Pulls pitch down and turns on claw
    Left Trigger --> Pulls pitch up both stages, outtake at bucket

     */

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);

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

        pitchController = new PIDFController(0.022, 0, 0.0004,0);
        //good
        slideController = new PIDFController(0.026, 0, 0.00026, 0.00012);
        //good
        pitchController2 = new PIDFController(0.012, 0, 0.0003,0);
        //good
        slideController2 = new PIDFController(0.04, 0, 0.0004, 0.0005);
        //good
        slidesBack = new OuttakeSlides(hardwareMap, "backSlide");
        slidesBack.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //slidesBack.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        clawBack = hardwareMap.get(CRServo.class, "backClaw");



        pitchBack1 = hardwareMap.get(Servo.class, "pitchBack1");
        pitchBack2 = hardwareMap.get(Servo.class, "pitchBack2");

        //hang motor
        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");



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

        if (isStopRequested()){
            return;
        }

        pitchMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesFront.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //pitchMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive() && !isStopRequested()) {

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
            hangMotor.setPower(gamepad2.right_stick_y);

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
            //lastPitchPower2 = pitchPower2;

            //==============FRONT OUTTAKE FST=============================

            switch(stageFront) {


                case IDLE: {

                    //visionSystem.getPortal().stopStreaming();

                    frontWristPos = 0.5;
                    changeWrist = false;
                    extensionPos = extensionInPos;

                    slideFrontTarget = 0;
                    pitchFrontTarget = 0;

                    pitchChange = 0;

                    //reset the encoder of the slide when at idle
                    slidesFront.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slidesFront.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    if (slidesFront.slideMotor.getCurrentPosition() < 2) {
                        slidesFront.slideMotor.setPower(0);
                    }

                    if (gamepad2.left_bumper) {
                        stageFront = Stages.SLIDEUP;
                        //stageFront = Stages.OUTTAKING;
                        outStageFront = Outtake.BUCKET;
                        slideFrontTarget = FRONT_SLIDE_HIGH_BASKET;
                    }

                    if (gamepad2.right_bumper) {
                        stageFront = Stages.SLIDEUP;
                        //stageFront = Stages.OUTTAKING;
                        outStageFront = Outtake.CHAMBER;
                        slideFrontTarget = FRONT_SLIDE_LOW_BASKET;
                    }

                    if (gamepad2.right_trigger > 0.05) {
                        stageFront = Stages.PITCHDOWN;
                        //stageFront = Stages.INTAKING;
                        inStageFront = Intake.ABOVE;
                        pitchFrontTarget = SAMPLE_ABOVE_PITCH_POS;
                        slideFrontTarget = 0; //stays zero
                    }

                    if (gamepad2.left_trigger > 0.05 && !(prevGamepad2.left_trigger > 0.05)) {
                        stageFront = Stages.PITCHDOWN;
                        //stageFront = Stages.INTAKING;
                        inStageFront = Intake.SPECIMEN_ABOVE;
                        pitchFrontTarget = SPECIMEN_ABOVE_PITCH_POS;
                    }

                    break;
                }


                case SLIDEUP: {

                    pitchFrontTarget = 0;
                    extensionPos = extensionOutPos;

                    frontWristPos = 0;

                    if (outStageFront == Outtake.BUCKET) {
                        slideFrontTarget = FRONT_SLIDE_HIGH_BASKET;
                    } else {
                        slideFrontTarget = FRONT_SLIDE_LOW_BASKET;
                    }

                    //outtake while the arm is going up
                    if (gamepad1.left_trigger > 0.05) {
                        clawFront.setPower(0.35);
                    } else {
                        clawFront.setPower(0);
                    }

//                    double power2 = pitchController.calculate(pitchMotorFront.getCurrentPosition(), pitchFrontTarget);
//                    pitchMotorFront.setPower(power2);

//                    telemetry.addData("Sliding Up", power2);
                    telemetry.addData("Pos", slidesFront.slideMotor.getCurrentPosition());

                    if ((Math.abs(slidesFront.slideMotor.getCurrentPosition() - slideFrontTarget) < 40)) {
                        stageFront = Stages.OUTTAKING;
                    }

                    break;
                }


                case OUTTAKING: {

                    extensionPos = extensionOutPos;

                    frontWristPos = 0;

                    if (outStageFront == Outtake.CHAMBER) {
                        pitchFrontTarget = 150;
                    }

                    if (gamepad1.left_trigger > 0.05) {
                        clawFront.setPower(0.35);
                    } else {
                        clawFront.setPower(0);
                    }

                    if (gamepad2.left_bumper) {
                        stageFront = Stages.SLIDEDOWN;
                        //stageFront = Stages.IDLE;
                        slideFrontTarget = 0;
                    }

                    if (gamepad2.right_bumper && slideFrontTarget == 1500) { //OLD STUFF I THINK?
                        slideFrontTarget = 750;
                    }


                    //while the slides are being brought down, keep the sample inside the claw
                    if (slidesFront.slideMotor.getVelocity() < -0.01) {
                        clawFront.setPower(-1);
                    }

                    break;
                }


                case SLIDEDOWN: {

                    extensionPos = extensionInPos;

                    telemetry.addData("Sliding Down", slidePower);
                    telemetry.addData("Sliding Velocity", slidesFront.slideMotor.getVelocity());
                    telemetry.addData("Pos", slidesFront.slideMotor.getCurrentPosition());

                    //if the motor cannot move, assume it has reached the bottom position
                    if (slidesFront.slideMotor.getVelocity() < 0.01 && slidesFront.slideMotor.getCurrentPosition() < 150 && slidePower < 0){
                        slidesFront.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slidesFront.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        stageFront = Stages.IDLE;
                    }

                    if (Math.abs(slidesFront.slideMotor.getCurrentPosition() - slideFrontTarget) < 20) {
                        stageFront = Stages.IDLE;
                    }

                    break;

                }


                case PITCHDOWN: {

                    extensionPos = extensionOutPos;

                    telemetry.addData("Pitching Down", pitchPower);
                    telemetry.addData("Pos", pitchMotorFront.getCurrentPosition());

                    if (inStageFront == Intake.ABOVE || inStageFront == Intake.SPECIMEN_ABOVE) {
                        clawFront.setPower(0);
                    } else {
                        clawFront.setPower(-1);
                    }

                    if (inStageFront == Intake.SPECIMEN_OUT) {
                        clawFront.setPower(1);
                    }

                    if ((Math.abs(pitchMotorFront.getCurrentPosition() - pitchFrontTarget) < 10)) {

//                        if (inStageFront == Intake.ABOVE) {
//                            visionSystem.getPortal().resumeStreaming();
//                        }

                        stageFront = Stages.INTAKING;
                    }

                    break;
                }


                case INTAKING: {

                    extensionPos = extensionOutPos;

                    switch (inStageFront) {

                        case ABOVE: {

                            slideFrontTarget = 0;

                            pitchFrontTarget = SAMPLE_ABOVE_PITCH_POS;

                            if (gamepad2.left_stick_x != 0) {
                                changeWrist = true;
                            }

                            if (changeWrist) {
                                frontWristPos = Range.clip(frontWristPos + (frontWristSpeed * gamepad2.left_stick_x), 0, 1);
                            }

//                            if (!changeWrist && wristTimer.milliseconds() > 300) {
//                                    frontWristPos = 1 - llDetector.getServoPos();
//                                    wristTimer.reset();
//                            }


//                            if (wristTimer.milliseconds() > 300) {
//                                if (visionSystem != null && visionSystem.getDetector() != null){
//                                    wristFront.setPosition(1 - visionSystem.getDetector().getServoPos());
////                                } else {
////                                  wristFront.setPosition(1 - visionSystem.getDetector().getServoPos());
//                                    wristTimer.reset();
//                                }
//                            }


                            //test
//                            if (visionSystem.getDetector() == null){
//                                telemetry.addData("Error", "Detector is null");
//                            }else {
//                                telemetry.addData("Detector", "Active");
//                            }
                            telemetry.addData("Timer", wristTimer.milliseconds());
                            telemetry.addData("wrist pos", wristFront.getPosition());
                            //eject unwanted samples
                            if (gamepad1.left_trigger > 0.05) {
                                clawFront.setPower(0.6);
                            } else {
                                clawFront.setPower(0);
                            }

                            if (gamepad2.right_trigger > 0.05 && !(prevGamepad2.right_trigger > 0.05)) {
                                inStageFront = Intake.PICKUP;
                                pitchFrontTarget = SAMPLE_PICKUP_PITCH_POS;
                                changeWrist = false;
                                //visionSystem.getPortal().stopStreaming();
                            }

                            if (gamepad2.left_trigger > 0.05 && !(prevGamepad2.left_trigger > 0.05) && pitchMotorFront.getVelocity(AngleUnit.RADIANS) < 0.1) {
                                inStageFront = Intake.IDLE;
                                stageFront = Stages.PITCHUP;
                                //stageFront = Stages.IDLE;
                                pitchFrontTarget = 0;
                                extensionPos = extensionInPos;
                                //visionSystem.getPortal().stopStreaming();
                            }

                            break;
                        }

                        case PICKUP: {

                            pitchChange += (int) gamepad2.left_stick_y;

                            pitchFrontTarget = SAMPLE_PICKUP_PITCH_POS + pitchChange;

                            slideFrontTarget = 0;


                            if (gamepad2.dpad_left) {
                                clawFront.setPower(1);
                                pitchFrontTarget = SAMPLE_OUT_PITCH_POS;
                            } else {
                                clawFront.setPower(-1);
                            }

                            if (gamepad2.left_trigger > 0.05) {
                                inStageFront = Intake.ABOVE;
                                //stageFront = Stages.PITCHUP;
                                pitchFrontTarget = SAMPLE_ABOVE_PITCH_POS;
                                //visionSystem.getPortal().resumeStreaming();
                            }

                            break;
                        }

                        case SPECIMEN_ABOVE: {

                            pitchFrontTarget = SPECIMEN_ABOVE_PITCH_POS;

                            //eject unwanted samples
                            if (gamepad1.left_trigger > 0.05) {
                                clawFront.setPower(0.6);
                            } else {
                                clawFront.setPower(0);
                            }

                            if (gamepad2.right_trigger > 0.05) {
                                inStageFront = Intake.SPECIMEN_PICKUP;
                                stageFront = Stages.PITCHDOWN;
                                pitchFrontTarget = SPECIMEN_PICKUP_PITCH_POS;
                            }

                            if (gamepad1.right_bumper) {
                                inStageFront = Intake.SPECIMEN_OUT;
                                stageFront = Stages.PITCHDOWN;
                                pitchFrontTarget = SPECIMEN_OUT_PITCH_POS;
                            }

                            if (gamepad2.left_trigger > 0.05 && !(prevGamepad2.left_trigger > 0.05)) {
                                inStageFront = Intake.IDLE;
                                stageFront = Stages.PITCHUP;
                                slideFrontTarget = 0;
                                pitchFrontTarget = 0;
                            }

                            break;
                        }

                        case SPECIMEN_PICKUP: {

                            pitchFrontTarget = SPECIMEN_PICKUP_PITCH_POS;

                            slideFrontTarget = 0;

                            clawFront.setPower(-1);



                            if (gamepad2.left_trigger > 0.05) {
                                inStageFront = Intake.SPECIMEN_ABOVE;
                                stageFront = Stages.PITCHUP;
                                pitchFrontTarget = SPECIMEN_ABOVE_PITCH_POS;
                                slideFrontTarget = 200;
                            }

                            break;
                        }

                        case SPECIMEN_OUT: {

                            pitchFrontTarget = SPECIMEN_OUT_PITCH_POS;

                            slideFrontTarget = 0;

                            clawFront.setPower(1);


                            if (gamepad1.right_bumper) {
                                inStageFront = Intake.SPECIMEN_ABOVE;
                                stageFront = Stages.PITCHUP;
                                pitchFrontTarget = SPECIMEN_ABOVE_PITCH_POS;
                                slideFrontTarget = 200;
                            }

                            break;
                        }

                    }


                    break;
                }


                case PITCHUP: {

                    switch(inStageFront) {
                        case IDLE: {
                            pitchFrontTarget = 0;
                            extensionPos = extensionInPos;
                            break;
                        }

                        case SPECIMEN_ABOVE: {
                            pitchFrontTarget = SPECIMEN_ABOVE_PITCH_POS;
                            break;
                        }

                        case ABOVE: {
                            pitchFrontTarget = SAMPLE_ABOVE_PITCH_POS;
                            break;
                        }
                    }

                    clawFront.setPower(0);

                    telemetry.addData("Pitching Up", pitchPower);
                    telemetry.addData("Pitch velo", pitchMotorFront.getVelocity());
                    telemetry.addData("Pos", pitchMotorFront.getCurrentPosition());

                    if (Math.abs(pitchMotorFront.getCurrentPosition() - pitchFrontTarget) < 5) {
                        if (inStageFront == Intake.IDLE) {
                            stageFront = Stages.IDLE;
                        } else {
                            stageFront = Stages.INTAKING;
                        }
                    }

                    //reset pitch encoder, same trick as the slides
                    if (Math.abs(pitchMotorFront.getVelocity()) <= 0.01  && pitchMotorFront.getCurrentPosition() < 20 && pitchPower < 0){
                        pitchMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pitchMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        if (inStageFront == Intake.IDLE) {
                            stageFront = Stages.IDLE;
                        } else {
                            stageFront = Stages.INTAKING;
                        }
                    }

                    break;
                }


            }

            //====FULL ROBOT RESET BUTTONS (IN CASE OF EMERGENCY)=============
            if (gamepad1.dpad_down) {
                slidesFront.slideMotor.setPower(-1);
                stageFront = Stages.IDLE;
                inStageFront = Intake.IDLE;
            }

            if (gamepad1.dpad_right) {
                slidesFront.slideMotor.setPower(1);
                stageFront = Stages.IDLE;
                inStageFront = Intake.IDLE;
            }

            if (gamepad1.dpad_up) {
                pitchMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pitchMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pitchMotorFront.setPower(-1);
                stageFront = Stages.IDLE;
                inStageFront = Intake.IDLE;
            }

            if (gamepad1.dpad_left) {
                pitchMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pitchMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pitchMotorFront.setPower(1);
                stageFront = Stages.IDLE;
                inStageFront = Intake.IDLE;
            }



//            //intake
//            if (gamepad1.dpad_left) {
//                clawFront.setPower(-1);
//            }
//            //outtake
//            if (gamepad1.dpad_right) {
//                clawFront.setPower(1);
//            }
//
//            //outtake
//            if (gamepad1.circle) {
//                clawBack.setPower(-1);
//            }

            //intake
            if (gamepad2.dpad_down) {
                clawFront.setPower(1);
            }



            //TODO:
            //==========END OF FRONT MECHANISM==================================
            //=========START OF BACK MECHANISM==================================
            //TODO:


            switch(stageBack) {


                case IDLE: {

                    slideBackTarget = 0;


                    clawBack.setPower(0);

                    pitchBackpos = BACK_PITCH_IDLE;

                    //reset the encoder of the slide when at idle
                    slidesBack.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //when slides are at bottom motor power is at 0
                    if (slidesBack.slideMotor.getCurrentPosition() < 2) {
                        slidesBack.slideMotor.setPower(0);
                    }


                    //??
                    if (gamepad2.share) {
                        stageBack = Stages.PITCHDOWN;
                        outStageBack = Outtake.HANG;
                    }

                    if (gamepad2.triangle) {
                        stageBack = Stages.SLIDEUP;
                        //stageBack = Stages.OUTTAKING;
                        outStageBack = Outtake.CHAMBER;

                        //for 312rpm motor
//                        slideBackTarget = 1800;

                        //for 435 rpm motor
                        slideBackTarget = BACK_SLIDE_HIGH_CHAMBER;
                    }

                    if (gamepad2.cross) {
                        stageBack = Stages.PITCHDOWN;
                        //stageBack = Stages.INTAKING;
                        inStageBack = Intake.ABOVE;
                        pitchBackpos = BACK_PITCH_ABOVE;

                        slideBackTarget = 0;
                    }

                    if (gamepad2.square && !(prevGamepad2.square)) {
                        stageBack = Stages.INTAKING;
                        //stageBack = Stages.INTAKING;
                        inStageBack = Intake.SPECIMEN_PICKUP;
                        //pitchBackTarget = (int) (SPECIMEN_ABOVE_PITCH_POS * 2.5);
                    }

                    break;
                }


                case SLIDEUP: {


                    pitchBackpos = BACK_PITCH_IDLE;
//                    changed

                    //for 312rpm motor
//                        slideBackTarget = 1800;

                    //for 435 rpm motor
                    slideBackTarget = BACK_SLIDE_HIGH_CHAMBER;
                    //??
//                    double power2 = pitchController2.calculate(pitchMotorBack.getCurrentPosition(), pitchBackTarget);
//                    pitchMotorBack.setPower(power2);

                    //telemetry.addData("Sliding Up", pitchPower2);
                    telemetry.addData("Pos", slidesBack.slideMotor.getCurrentPosition());

                    if ((Math.abs(slidesBack.slideMotor.getCurrentPosition() - slideBackTarget) < 40)) {
                    stageBack = Stages.OUTTAKING;
                    } //changed

                    break;
                }


                case OUTTAKING: {

                    if (outStageBack == Outtake.CHAMBER) {
                        pitchBackpos = BACK_PITCH_CHAMBER;
                    }

                    //eject samples
                    if (gamepad1.right_trigger > 0.05) {
                        clawBack.setPower(-0.7);
                    } else {
                        clawBack.setPower(0);
                    }


                    if (gamepad2.triangle && !prevGamepad2.triangle) {
                        stageBack = Stages.SLIDEDOWN;
                        //stageBack = Stages.IDLE;
                        slideBackTarget = BACK_SLIDE_HIGH_CHAMBER;
                        clawBack.setPower(-0.25);
                    }

                    //unnecessary
//                    if (gamepad2.circle && slideBackTarget == 1900) {
//                        stageBack = Stages.SLIDEDOWN;
//                        slideBackTarget = 1250;
//
////                        slideBackTarget = 750;
//                        //old
//                    }


                    //while the slides are being brought down, keep the sample inside the claw
//                    if (slidesBack.slideMotor.getVelocity() < -0.01 && outStageBack != Outtake.HANG) {
//                        clawBack.setPower(-1);
//                    }

                    if (outStageBack == Outtake.HANG) {
                        //ACTUAL HANGING FOR ROBOT
                        pitchBackpos = BACK_PITCH_CHAMBER;
                        slideBackTarget = 625;

                        if (Math.abs(slidesBack.slideMotor.getCurrentPosition() - 900) < 30) {
                            pitchFrontTarget = 0;
                        }

                    }

                    break;
                }


                case SLIDEDOWN: {

                    telemetry.addData("Sliding Down", slidePower);
                    telemetry.addData("Sliding Velocity", slidesBack.slideMotor.getVelocity());
                    telemetry.addData("Pos", slidesBack.slideMotor.getCurrentPosition());

                    if (slidesBack.slideMotor.getCurrentPosition() < 400) {
                        pitchBackpos = BACK_PITCH_IDLE;
                    }

                    slideBackTarget = 0;

                    //if the motor cannot move, assume it has reached the bottom position
                    if (slidesBack.slideMotor.getVelocity() < 0.05 && slidesBack.slideMotor.getCurrentPosition() < 100 && slidePower <= 0){
                        slidesBack.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        stageBack = Stages.IDLE;
                    }
                        //reset back arm at 0
                    if (Math.abs(slidesBack.slideMotor.getCurrentPosition() - slideBackTarget) < 30 && slideBackTarget <= 35) { //changed
                        slidesBack.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        stageBack = Stages.IDLE;
                    } //changed

                    break;

                }


                case PITCHDOWN: {

                    //telemetry.addData("Pitching Down", pitchPower2);
                    //telemetry.addData("Pos", pitchMotorBack.getCurrentPosition());

                    if (inStageBack == Intake.ABOVE || inStageBack == Intake.SPECIMEN_ABOVE) {
                        clawBack.setPower(0);
                    } else {
                        clawBack.setPower(1);
                    }

                    if (pitchBack1.getPosition() == BACK_PITCH_ABOVE) {
                        stageBack = Stages.INTAKING;
                    }

                    break;
                }


                case INTAKING: {

                    switch (inStageBack) {

                        case ABOVE: {

                            slideBackTarget = 0;
//                            slideBackTarget = SAMPLE_ABOVE_SLIDE_POS; changed

                            pitchBackpos = BACK_PITCH_ABOVE;

                            //eject unwanted samples
                            if (gamepad1.right_trigger > 0.05) {
                                clawBack.setPower(-0.7);
                            } else {
                                clawBack.setPower(0);
                            }

                            if (gamepad2.cross && !(prevGamepad2.cross)) {
                                inStageBack = Intake.PICKUP;
                                pitchBackpos = BACK_PITCH_PICKUP;
                            }

                            if (gamepad2.circle && !(prevGamepad2.circle)) {
                                inStageBack = Intake.IDLE;
                                stageBack = Stages.PITCHUP;
                                //stageBack = Stages.IDLE;
                                pitchBackpos = BACK_PITCH_IDLE;
                            }

                            break;
                        }

                        case PICKUP: {

                            pitchBackpos = BACK_PITCH_PICKUP;

                            slideBackTarget = 0;
//                            slideBackTarget = SAMPLE_PICKUP_SLIDE_POS; changed

                            if (gamepad2.dpad_right) {
                                clawBack.setPower(-1);
                            } else {
                                clawBack.setPower(1);
                            }

                            if (gamepad2.circle) {
                                inStageBack = Intake.ABOVE;
                                stageBack = Stages.PITCHUP;
                                pitchBackpos = BACK_PITCH_ABOVE;
                            }

                            break;
                        }

                        case SPECIMEN_ABOVE: {

                            pitchBackpos = BACK_PITCH_WALL;
                            //weird might have to change to back pitch wall, original was idle

                            //eject unwanted samples
                            if (gamepad1.right_trigger > 0.05) {
                                clawBack.setPower(-0.7);
                            } else {
                                clawBack.setPower(0);
                            }

//                            if (gamepad2.circle && !prevGamepad2.circle) {
//                                inStageBack = Intake.SPECIMEN_PICKUP;
//                                //stageBack = Stages.PITCHDOWN;
//                                pitchBackTarget = 0; //(int) (SPECIMEN_PICKUP_PITCH_POS * 2.5);
//                            }

                            if (gamepad2.square && !(prevGamepad2.square)) {
                                inStageBack = Intake.IDLE;
                                stageBack = Stages.PITCHUP;
                                slideBackTarget = 0;
                                pitchBackpos = BACK_PITCH_IDLE;
                            }

                            if (gamepad2.triangle) {
                                inStageBack =  Intake.IDLE;
                                stageBack = Stages.SLIDEUP;
                                outStageBack = Outtake.CHAMBER;

                                slideBackTarget = BACK_SLIDE_HIGH_CHAMBER;
//                                slideBackTarget = 950; changed

                                pitchBackpos = BACK_PITCH_CHAMBER;
                                //stage back chamber
                            }

                            break;
                        }

                        case SPECIMEN_PICKUP: {

                            pitchBackpos = BACK_PITCH_WALL;

                            //slideBackTarget = 0;

                            clawBack.setPower(1);

                            if (gamepad2.circle) {
                                inStageBack = Intake.SPECIMEN_ABOVE;
                                //stageBack = Stages.PITCHUP;
                                //weird might have to change to back pitch idle
                                slideBackTarget = 120;
                                if(slideBackTarget - slidesBack.slideMotor.getCurrentPosition() <= 20) {
                                    pitchBackpos = BACK_PITCH_IDLE;
                                }
                                //raise specimen off wall
                            }

                            break;
                        }

                    }


                    break;
                }


                case PITCHUP: {

                    switch(inStageBack) {
                        case IDLE: {
                            pitchBackpos = BACK_PITCH_IDLE;
                            break;
                        }

                        case SPECIMEN_ABOVE: {
                            pitchBackpos = 0.15;
                            break;
                        }

                        case ABOVE: {
                            pitchBackpos = BACK_PITCH_ABOVE;
                            break;
                        }
                    }

                    clawBack.setPower(0);

                    telemetry.addData("Pitching Up", pitchPower);
                    //telemetry.addData("Pos", pitchMotorBack.getCurrentPosition());

                    if (Math.abs(pitchBack1.getPosition() - pitchBackpos) < 0.01) {
                        if (inStageBack == Intake.IDLE) {
                            stageBack = Stages.IDLE;
                        } else {
                            stageBack = Stages.INTAKING;
                        }
                    }

                    break;
                }


            }



            //ALL FAIL SAFES START HERE

            //gamepad 2 failsafes


//            //intake
//            if (gamepad2.dpad_up) {
//                clawFront.setPower(-1);
//            }
            //outtake
            if (gamepad2.dpad_up) {
                clawBack.setPower(-1);
            }
//
//            //intake
//            if (gamepad2.dpad_right) {
//                clawBack.setPower(1);
//            }

            //gamepad 1 failsafes

            //resets front arm and brings it down
            if (gamepad1.triangle) {

            }

            //intake
            if (gamepad1.square) {
                clawBack.setPower(-1);
            }

            if (gamepad1.circle) {
                clawBack.setPower(1);
            }

            //resets front arm height
            if (gamepad1.cross) {
                slidesBack.slideMotor.setPower(-1);
                stageBack = Stages.IDLE;
                inStageBack = Intake.IDLE;
            }




            //FtcDashboard.getInstance().sendImage(visionSystem.getDetector().getLastFrame());
            telemetry.addData("Loop Time (ms)", loopTime.milliseconds());
            loopTime.reset();
            telemetry.addData("Wrist Pos", frontWristPos);
            telemetry.addData("Extension Pos", extensionPos);
            telemetry.addData("RightExt Pos", rightExt.getPosition());
            telemetry.addData("LeftExt Pos", leftExt.getPosition());

            //telemetry.addData("Servo Pos", llDetector.getServoPos());

            //telemetry.addData("Pitch Angle (rad)", Math.cos((pitchMotorFront.getCurrentPosition())/145.1 * 28 * Math.PI));

            telemetry.addData("Front Slide Position", slidesFront.slideMotor.getCurrentPosition());
            telemetry.addData("Front Slide Power", slidePower);
            telemetry.addData("Front Pitch Position", pitchMotorFront.getCurrentPosition());
            telemetry.addData("Front Pitch Power", pitchPower);
            telemetry.addData("Front Slide Target", slideFrontTarget);
            telemetry.addData("Front Pitch Target", pitchFrontTarget);
            telemetry.addData("","");
            telemetry.addData("Back Slide Position", slidesBack.slideMotor.getCurrentPosition());
            //telemetry.addData("Back Pitch Position", pitchMotorBack.getCurrentPosition());
            telemetry.addData("Back Slide Target", slideBackTarget);
            telemetry.addData("Back Pitch Position", pitchBack1.getPosition());
            telemetry.addData("Back Pitch Position Target Servo", pitchBackpos);
            telemetry.addData("Hang Power", hangMotor.getPower());
            //hang motor


            telemetry.addData("","");
            telemetry.addData("","");


            telemetry.addData("StageFront", stageFront);
            telemetry.addData("Intake Stage Front", inStageFront);
            telemetry.addData("Outtake Stage Front", outStageFront);

            telemetry.addData("------", "------");

            telemetry.addData("Back Slide Position", slidesBack.slideMotor.getCurrentPosition());
            //telemetry.addData("Back Pitch Position", pitchMotorBack.getCurrentPosition());

            telemetry.addData("StageBack", stageBack);
            telemetry.addData("Intake Stage Back", inStageBack);
            telemetry.addData("Outtake Stage Back", outStageBack);
            telemetry.update();


        }



    }
}