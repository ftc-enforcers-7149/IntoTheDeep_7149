package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;


@TeleOp(name = "FST TeleOp Main")
public class FSTTeleop_TwoClaws extends LinearOpMode {


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

    private enum Stages {
        IDLE, INTAKING, OUTTAKING, SLIDEUP, SLIDEDOWN, PITCHUP, PITCHDOWN
    }

    private enum Outtake {
        BUCKET, CHAMBER
    }

    private enum Intake {
        ABOVE, PICKUP, IDLE, SPECIMEN_ABOVE, SPECIMEN_PICKUP
    }

    Stages stageFront = Stages.IDLE;
    Outtake outStageFront = Outtake.BUCKET;
    Intake inStageFront = Intake.IDLE;

    Stages stageBack = Stages.IDLE;
    Outtake outStageBack = Outtake.BUCKET;
    Intake inStageBack = Intake.IDLE;

    Gamepad prevGamepad2, currentGamepad2;


    //Old sample above: 880, 1150
    final int SAMPLE_ABOVE_PITCH_POS = 975, SAMPLE_PICKUP_PITCH_POS = 1050;
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

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);

        slidesFront = new OuttakeSlides(hardwareMap, "frontSlide");
        slidesFront.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawFront = hardwareMap.get(CRServo.class, "frontClaw");
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
            pitchMotorFront.setPower(pitchPower);

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

            //==============FRONT OUTTAKE FST=============================

            switch(stageFront) {


                case IDLE: {

                    slideFrontTarget = 0;
                    pitchFrontTarget = 0;

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
                        slideFrontTarget = 2600;
                    }

                    if (gamepad2.right_bumper) {
                        stageFront = Stages.SLIDEUP;
                        //stageFront = Stages.OUTTAKING;
                        outStageFront = Outtake.CHAMBER;
                        slideFrontTarget = 1200;
                    }

                    if (gamepad2.right_trigger > 0.05) {
                        stageFront = Stages.PITCHDOWN;
                        //stageFront = Stages.INTAKING;
                        inStageFront = Intake.ABOVE;
                        pitchFrontTarget = SAMPLE_ABOVE_PITCH_POS;
                        slideFrontTarget = 0;
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

                    if (outStageFront == Outtake.BUCKET) {
                        slideFrontTarget = 2600;
                    } else {
                        slideFrontTarget = 1200;
                    }

                    double power2 = pitchController.calculate(pitchMotorFront.getCurrentPosition(), pitchFrontTarget);
                    pitchMotorFront.setPower(power2);

                    telemetry.addData("Sliding Up", power2);
                    telemetry.addData("Pos", slidesFront.slideMotor.getCurrentPosition());

                    if ((Math.abs(slidesFront.slideMotor.getCurrentPosition() - slideFrontTarget) < 40)) {
                        stageFront = Stages.OUTTAKING;
                    }

                    break;
                }


                case OUTTAKING: {

                    if (outStageFront == Outtake.CHAMBER) {
                        pitchFrontTarget = 150;
                    }

                    if (gamepad1.left_trigger > 0.05) {
                        clawFront.setPower(0.3);
                    } else {
                        clawFront.setPower(0);
                    }

                    if (gamepad2.left_bumper) {
                        stageFront = Stages.SLIDEDOWN;
                        //stageFront = Stages.IDLE;
                        slideFrontTarget = 0;
                    }

                    if (gamepad2.right_bumper && slideFrontTarget == 1200) {
                        slideFrontTarget = 750;
                    }


                    //while the slides are being brought down, keep the sample inside the claw
                    if (slidesFront.slideMotor.getVelocity() < -0.01) {
                        clawFront.setPower(-1);
                    }

                    break;
                }


                case SLIDEDOWN: {

                    telemetry.addData("Sliding Down", slidePower);
                    telemetry.addData("Sliding Velocity", slidesFront.slideMotor.getVelocity());
                    telemetry.addData("Pos", slidesFront.slideMotor.getCurrentPosition());

                    //if the motor cannot move, assume it has reached the bottom position
                    if (slidesFront.slideMotor.getVelocity() < 0.01 && slidesFront.slideMotor.getCurrentPosition() < 100 && slidePower < 0){
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

                    telemetry.addData("Pitching Down", pitchPower);
                    telemetry.addData("Pos", pitchMotorFront.getCurrentPosition());

                    if (inStageFront == Intake.ABOVE || inStageFront == Intake.SPECIMEN_ABOVE) {
                        clawFront.setPower(0);
                    } else {
                        clawFront.setPower(-1);
                    }

                    if ((Math.abs(pitchMotorFront.getCurrentPosition() - pitchFrontTarget) < 10)) {
                        stageFront = Stages.INTAKING;
                    }

                    break;
                }


                case INTAKING: {

                    switch (inStageFront) {

                        case ABOVE: {

                            slideFrontTarget = SAMPLE_ABOVE_SLIDE_POS;

                            pitchFrontTarget = SAMPLE_ABOVE_PITCH_POS;

                            //eject unwanted samples
                            if (gamepad1.left_trigger > 0.05) {
                                clawFront.setPower(0.6);
                            } else {
                                clawFront.setPower(0);
                            }

                            if (gamepad2.right_trigger > 0.05 && !(prevGamepad2.right_trigger > 0.05)) {
                                inStageFront = Intake.PICKUP;
                                pitchFrontTarget = SAMPLE_PICKUP_PITCH_POS;
                            }

                            if (gamepad2.left_trigger > 0.05 && !(prevGamepad2.left_trigger > 0.05) && pitchMotorFront.getVelocity(AngleUnit.RADIANS) < 0.1) {
                                inStageFront = Intake.IDLE;
                                stageFront = Stages.PITCHUP;
                                //stageFront = Stages.IDLE;
                                pitchFrontTarget = 0;
                            }

                            break;
                        }

                        case PICKUP: {

                            pitchFrontTarget = SAMPLE_PICKUP_PITCH_POS;

                            slideFrontTarget = SAMPLE_PICKUP_SLIDE_POS;

                            if (gamepad2.dpad_left) {
                                clawFront.setPower(1);
                            } else {
                                clawFront.setPower(-1);
                            }

                            if (gamepad2.left_trigger > 0.05) {
                                inStageFront = Intake.ABOVE;
                                //stageFront = Stages.PITCHUP;
                                pitchFrontTarget = SAMPLE_ABOVE_PITCH_POS;
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

                    }


                    break;
                }


                case PITCHUP: {

                    switch(inStageFront) {
                        case IDLE: {
                            pitchFrontTarget = 0;
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
                    telemetry.addData("Pos", pitchMotorFront.getCurrentPosition());

                    if (Math.abs(pitchMotorFront.getCurrentPosition() - pitchFrontTarget) < 30) {
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

            if (gamepad1.dpad_up) {
                pitchMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pitchMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pitchMotorFront.setPower(-1);
                stageFront = Stages.IDLE;
                inStageFront = Intake.IDLE;
            }

            //intake
            if (gamepad1.dpad_left) {
                clawFront.setPower(-1);
            }
            //outtake
            if (gamepad1.dpad_right) {
                clawFront.setPower(1);
            }
            //intake
            if (gamepad1.square) {
                clawBack.setPower(1);
            }
            //outtake
            if (gamepad1.circle) {
                clawBack.setPower(-1);
            }



            //TODO:
            //==========END OF FRONT MECHANISM==================================
            //=========START OF BACK MECHANISM==================================
            //TODO:


            switch(stageBack) {


                case IDLE: {

                    slideBackTarget = 0;
                    pitchBackTarget = 0;

                    //reset the encoder of the slide when at idle
                    slidesBack.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//                    if (gamepad2.triangle) {
//                        stageBack = Stages.SLIDEUP;
//                        //stageBack = Stages.OUTTAKING;
//                        outStageBack = Outtake.BUCKET;
//                        slideBackTarget = 1200;
//                    }

                    if (gamepad2.triangle) {
                        stageBack = Stages.SLIDEUP;
                        //stageBack = Stages.OUTTAKING;
                        outStageBack = Outtake.CHAMBER;
                        slideBackTarget = 1300;
                    }

                    if (gamepad2.cross) {
                        stageBack = Stages.PITCHDOWN;
                        //stageBack = Stages.INTAKING;
                        inStageBack = Intake.ABOVE;
                        pitchBackTarget = (int) (SAMPLE_ABOVE_PITCH_POS * backMotorOffsetAbove);
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

                    pitchBackTarget = 0;

                    slideBackTarget = 1300;

                    double power2 = pitchController2.calculate(pitchMotorBack.getCurrentPosition(), pitchBackTarget);
                    pitchMotorBack.setPower(power2);

                    telemetry.addData("Sliding Up", power2);
                    telemetry.addData("Pos", slidesBack.slideMotor.getCurrentPosition());

                    if ((Math.abs(slidesBack.slideMotor.getCurrentPosition() - slideBackTarget) < 40)) {
                        stageBack = Stages.OUTTAKING;
                    }

                    break;
                }


                case OUTTAKING: {

                    if (outStageBack == Outtake.CHAMBER) {
                        pitchBackTarget = 300;
                    }

                    if (gamepad1.right_trigger > 0.05) {
                        clawBack.setPower(0.3);
                    } else {
                        clawBack.setPower(0);
                    }

                    if (gamepad2.triangle) {
                        stageBack = Stages.SLIDEDOWN;
                        //stageBack = Stages.IDLE;
                        slideBackTarget = 0;
                    }

                    if (gamepad2.circle && slideBackTarget == 1300) {
                        slideBackTarget = 750;
                    }


                    //while the slides are being brought down, keep the sample inside the claw
                    if (slidesBack.slideMotor.getVelocity() < -0.01) {
                        clawBack.setPower(-1);
                    }

                    break;
                }


                case SLIDEDOWN: {

                    telemetry.addData("Sliding Down", slidePower);
                    telemetry.addData("Sliding Velocity", slidesBack.slideMotor.getVelocity());
                    telemetry.addData("Pos", slidesBack.slideMotor.getCurrentPosition());

                    //if the motor cannot move, assume it has reached the bottom position
                    if (slidesBack.slideMotor.getVelocity() < 0.01 && slidesBack.slideMotor.getCurrentPosition() < 100 && slidePower < 0){
                        slidesBack.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slidesBack.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        stageBack = Stages.IDLE;
                    }

                    if (Math.abs(slidesBack.slideMotor.getCurrentPosition() - slideBackTarget) < 20) {
                        stageBack = Stages.IDLE;
                    }

                    break;

                }


                case PITCHDOWN: {

                    telemetry.addData("Pitching Down", pitchPower2);
                    telemetry.addData("Pos", pitchMotorBack.getCurrentPosition());

                    if (inStageBack == Intake.ABOVE || inStageBack == Intake.SPECIMEN_ABOVE) {
                        clawBack.setPower(0);
                    } else {
                        clawBack.setPower(1);
                    }

                    if ((Math.abs(pitchMotorBack.getCurrentPosition() - pitchBackTarget) < 10)) {
                        stageBack = Stages.INTAKING;
                    }

                    break;
                }


                case INTAKING: {

                    switch (inStageBack) {

                        case ABOVE: {

                            slideBackTarget = SAMPLE_ABOVE_SLIDE_POS;

                            pitchBackTarget = (int) (SAMPLE_ABOVE_PITCH_POS * backMotorOffsetAbove);

                            //eject unwanted samples
                            if (gamepad1.right_trigger > 0.05) {
                                clawBack.setPower(0.7);
                            } else {
                                clawBack.setPower(0);
                            }

                            if (gamepad2.cross && !(prevGamepad2.cross)) {
                                inStageBack = Intake.PICKUP;
                                pitchBackTarget = (int) (SAMPLE_PICKUP_PITCH_POS * backMotorOffsetDown);
                            }

                            if (gamepad2.circle && !(prevGamepad2.circle)) {
                                inStageBack = Intake.IDLE;
                                stageBack = Stages.PITCHUP;
                                //stageBack = Stages.IDLE;
                                pitchBackTarget = 0;
                            }

                            break;
                        }

                        case PICKUP: {

                            pitchBackTarget = (int) (SAMPLE_PICKUP_PITCH_POS * backMotorOffsetDown);

                            slideBackTarget = SAMPLE_PICKUP_SLIDE_POS;

                            if (gamepad2.dpad_right) {
                                clawBack.setPower(-1);
                            } else {
                                clawBack.setPower(1);
                            }

                            if (gamepad2.circle) {
                                inStageBack = Intake.ABOVE;
                                stageBack = Stages.PITCHUP;
                                pitchBackTarget = (int) (SAMPLE_ABOVE_PITCH_POS * backMotorOffsetAbove);
                            }

                            break;
                        }

                        case SPECIMEN_ABOVE: {

                            pitchBackTarget = 0; //(int) (SPECIMEN_ABOVE_PITCH_POS * 2.5);

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
                                pitchBackTarget = 0;
                            }

                            if (gamepad2.triangle) {
                                inStageBack =  Intake.IDLE;
                                stageBack = Stages.SLIDEUP;
                                outStageBack = Outtake.CHAMBER;
                                slideBackTarget = 1100;
                                pitchBackTarget = 200;
                            }

                            break;
                        }

                        case SPECIMEN_PICKUP: {

                            pitchBackTarget = 0; //(int) (SPECIMEN_PICKUP_PITCH_POS * 2.5);

                            slideBackTarget = 0;

                            clawBack.setPower(1);

                            if (gamepad2.circle) {
                                inStageBack = Intake.SPECIMEN_ABOVE;
                                //stageBack = Stages.PITCHUP;
                                pitchBackTarget = 0; //(int) (SPECIMEN_ABOVE_PITCH_POS * 2.5);
                                slideBackTarget = 200;
                            }

                            break;
                        }

                    }


                    break;
                }


                case PITCHUP: {

                    switch(inStageBack) {
                        case IDLE: {
                            pitchBackTarget = 0;
                            break;
                        }

                        case SPECIMEN_ABOVE: {
                            pitchBackTarget = (int) (SPECIMEN_ABOVE_PITCH_POS * 2.5);
                            break;
                        }

                        case ABOVE: {
                            pitchBackTarget = (int) (SAMPLE_ABOVE_PITCH_POS * 2.5);
                            break;
                        }
                    }

                    clawBack.setPower(0);

                    telemetry.addData("Pitching Up", pitchPower);
                    telemetry.addData("Pos", pitchMotorBack.getCurrentPosition());

                    if (Math.abs(pitchMotorBack.getCurrentPosition() - pitchBackTarget) < 30) {
                        if (inStageBack == Intake.IDLE) {
                            stageBack = Stages.IDLE;
                        } else {
                            stageBack = Stages.INTAKING;
                        }
                    }

                    break;
                }


            }

            //intake
            if (gamepad2.dpad_up) {
                clawFront.setPower(-1);
            }
            //outtake
            if (gamepad2.dpad_down) {
                clawFront.setPower(1);
            }
            //outtake
            if (gamepad2.dpad_left) {
                clawBack.setPower(-1);
            }
            //intake
            if (gamepad2.dpad_right) {
                clawBack.setPower(1);
            }

            telemetry.addData("Pitch Angle (rad)", Math.cos((pitchMotorFront.getCurrentPosition())/145.1 * 28 * Math.PI));

            telemetry.addData("Front Slide Position", slidesFront.slideMotor.getCurrentPosition());
            telemetry.addData("Front Slide Power", slidePower);
            telemetry.addData("Front Pitch Position", pitchMotorFront.getCurrentPosition());
            telemetry.addData("Front Pitch Power", pitchPower);
            telemetry.addData("Front Slide Target", slideFrontTarget);
            telemetry.addData("Front Pitch Target", pitchFrontTarget);
            telemetry.addData("","");
            telemetry.addData("Back Slide Position", slidesBack.slideMotor.getCurrentPosition());
            telemetry.addData("Back Pitch Position", pitchMotorBack.getCurrentPosition());
            telemetry.addData("Back Slide Target", slideBackTarget);
            telemetry.addData("Back Pitch Target", pitchBackTarget);

            telemetry.addData("","");
            telemetry.addData("","");


            telemetry.addData("StageFront", stageFront);
            telemetry.addData("Intake Stage Front", inStageFront);
            telemetry.addData("Outtake Stage Front", outStageFront);

            telemetry.addData("------", "------");

            telemetry.addData("Back Slide Position", slidesBack.slideMotor.getCurrentPosition());
            telemetry.addData("Back Pitch Position", pitchMotorBack.getCurrentPosition());

            telemetry.addData("StageBack", stageBack);
            telemetry.addData("Intake Stage Back", inStageBack);
            telemetry.addData("Outtake Stage Back", outStageBack);
            telemetry.update();


        }



    }
}
