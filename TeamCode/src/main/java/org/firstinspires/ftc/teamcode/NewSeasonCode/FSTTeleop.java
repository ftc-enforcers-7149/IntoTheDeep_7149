package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;

@TeleOp(name = "FST TeleOp")
public class FSTTeleop extends LinearOpMode {


    OuttakeSlides slides;
    CRServo claw;
    DcMotorEx pitchMotor;

    MecanumPowerDrive drive;

    int slideTarget = 0;
    int pitchTarget = 0;

    int slideInitPos;
    int pitchInitPos;

    private enum Stages {
        IDLE, INTAKING, OUTTAKING, SLIDEUP, SLIDEDOWN, PITCHUP, PITCHDOWN
    }

    private enum Outtake {
        BUCKET, CHAMBER
    }

    private enum Inttake {
        ABOVE, PICKUP, IDLE
    }

    Stages stage = Stages.IDLE;
    Outtake outStage = Outtake.BUCKET;
    Inttake inStage = Inttake.IDLE;

    /*

    Right Trigger --> Pulls pitch down to inttake --> Pulls pitch down and turns on claw
    Left Trigger --> Pulls pitch up both stages, outtake at bucket

     */

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);

        slides = new OuttakeSlides(hardwareMap, "slideMotor");
        claw = hardwareMap.get(CRServo.class, "claw");
        pitchMotor = hardwareMap.get(DcMotorEx.class, "pitch");

        slideInitPos = slides.initialPos;
        pitchInitPos = pitchMotor.getCurrentPosition();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while(opModeIsActive() && !isStopRequested()) {

            drive.setRobotCentricPower(gamepad1.left_stick_x * 1.08, -gamepad1.left_stick_y * 1.08, gamepad1.right_stick_x * 1.08);

            switch(stage) {


                case IDLE: {

                    slides.slideMotor.setPower(0);
                    pitchMotor.setPower(0);

                    if (gamepad1.left_bumper) {
                        stage = Stages.SLIDEUP;
                        outStage = outStage.BUCKET;
                        slideTarget = 400;
                    }

                    if (gamepad1.right_bumper) {
                        stage = Stages.SLIDEUP;
                        outStage = outStage.CHAMBER;
                        slideTarget = 300;
                    }

                    if (gamepad1.right_trigger > 0.1) {
                        stage = Stages.PITCHDOWN;
                        inStage = Inttake.ABOVE;
                        pitchTarget = 750;
                    }

                    break;
                }


                case SLIDEUP: {

                    pitchTarget = 0;

                    slides.slideMotor.setTargetPosition(slideTarget - slideInitPos);
                    slides.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    pitchMotor.setTargetPosition(pitchTarget - pitchInitPos);
                    pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if ((Math.abs(slides.slideMotor.getCurrentPosition() - slides.slideMotor.getTargetPosition()) < 20)
                            && (Math.abs(pitchMotor.getCurrentPosition() - pitchMotor.getTargetPosition()) < 20)) {
                        stage = Stages.OUTTAKING;
                    }

                    break;
                }


                case OUTTAKING: {

                    //setPower to maintain position, use PID once tuned
                    switch (outStage) {
                        case BUCKET: {
                            slides.slideMotor.setPower(0.15);
                            break;
                        }

                        case CHAMBER:{
                            slides.slideMotor.setPower(0.8);
                            break;
                        }
                    }

                    if (gamepad1.left_trigger > 0.1) {
                        claw.setPower(1);
                    } else {
                        claw.setPower(0);
                    }

                    if (gamepad1.right_bumper) {
                        stage = Stages.SLIDEDOWN;
                        slideTarget = 0;
                    }

                    break;
                }


                case SLIDEDOWN: {

                    slides.slideMotor.setTargetPosition(slideTarget - slideInitPos);
                    slides.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (Math.abs(slides.slideMotor.getCurrentPosition() - slides.slideMotor.getTargetPosition()) < 20) {
                        stage = Stages.IDLE;
                    }

                    break;

                }


                case PITCHDOWN: {

                    pitchMotor.setTargetPosition(pitchTarget - pitchInitPos);
                    pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (inStage == Inttake.ABOVE) {
                        claw.setPower(0);
                    } else {
                        claw.setPower(-1);
                    }

                    if (Math.abs(pitchMotor.getCurrentPosition() - pitchMotor.getTargetPosition()) < 20) {
                        stage = Stages.INTAKING;
                    }

                    break;
                }


                case INTAKING: {

                    switch (inStage) {

                        case ABOVE: {

                            if (gamepad1.triangle) {
                                claw.setPower(1);
                            } else {
                                claw.setPower(0);
                            }

                            if (gamepad1.right_trigger > 0.1) {
                                inStage = Inttake.PICKUP;
                                stage = Stages.PITCHDOWN;
                                pitchTarget = 850;
                            }

                            if (gamepad1.left_trigger > 0.1) {
                                inStage = Inttake.IDLE;
                                stage = Stages.PITCHUP;
                                pitchTarget = 0;
                            }

                            break;
                        }

                        case PICKUP: {
                            claw.setPower(-1);

                            if (gamepad1.left_trigger > 0.1) {
                                inStage = Inttake.ABOVE;
                                stage = Stages.PITCHUP;
                                pitchTarget = 750;
                            }

                            break;
                        }

                    }


                    break;
                }


                case PITCHUP: {

                    pitchMotor.setTargetPosition(pitchTarget - pitchInitPos);
                    pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    claw.setPower(0);

                    if (Math.abs(pitchMotor.getCurrentPosition() - pitchMotor.getTargetPosition()) < 20) {
                        if (inStage == Inttake.IDLE) {
                            stage = Stages.IDLE;
                        } else {
                            stage = Stages.INTAKING;
                        }
                    }

                    break;
                }


            }

            telemetry.addData("Slide Position", slides.slideMotor.getCurrentPosition());
            telemetry.addData("Pitch Position", pitchMotor.getCurrentPosition());

            telemetry.addData("Stage", stage);
            telemetry.addData("Intake Stage", inStage);
            telemetry.addData("Outtake Stage", outStage);
            telemetry.update();


        }



    }
}
