package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EndAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.InstantAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TimedAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.WaitAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.GlobalData.AutoConstants;
import org.firstinspires.ftc.teamcode.GlobalData.HardwareConstants;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.PeriodicFollower;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.ColorClaw;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.FourServoPitchArm;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.TwoMotorSlides;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Autonomous(name = "Basket Auto")
public class LeftBasket extends LinearOpMode {

    ActionManager actionManager;
    Follower follower;
    PeriodicFollower perFollower;

    TwoMotorSlides frontSlides;
    OuttakeSlides backSlides;
    FourServoPitchArm frontArm;
    ServoImplEx rightExt, leftExt;
    Servo pitchBack1, pitchBack2;
    ColorClaw colorClaw;

    Servo wristFront;

    IMU imu;

    EventAction slidesUpBasket, slidesDownBasket,
            armDownSample, armAboveSample, armUpSample,
            extensionOut, extensionIn,
            imuReset,
            slowFollower, quickFollower, medFollower,
            wristDropoff, wristPickup, wristPickupRot,
            subPitch,
            parkSlidesUp, parkPitch;

    ClawRotateAction clawOuttake, clawIntake;

    PedroAction moveBasket1, moveSample2, moveBasket2,
            moveSample3, moveBasket3, moveSample4, moveAwaySample4, moveBasket4,
            moveSub1, moveBasket5,
            park;

    Path basketPath1, samplePath2, basketPath2, samplePath3, basketPath3,
            samplePath4, awaySamplePath4, basketPath4, subPath1, basketPath5,
            parkPath;

    double subXOffset = 0, subYOffset = 0, wristOffset = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        frontSlides = new TwoMotorSlides(this);
        frontArm = new FourServoPitchArm(this);
        frontArm.setPosition(HardwareConstants.PITCH_ZERO);
        backSlides = new OuttakeSlides(hardwareMap, "backSlide");
        //backArm = new PitchArm(hardwareMap, "backPitch");

        frontSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backSlides.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlides.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backArm.pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backArm.pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitchBack1 = hardwareMap.get(Servo.class, "pitchBack1");
        pitchBack2 = hardwareMap.get(Servo.class, "pitchBack2");
        pitchBack1.setPosition(0);
        pitchBack2.setPosition(0);

        colorClaw = new ColorClaw(this);


        //backSlides.setPIDFCoefficients(0.08, 0, 0.00026, 0.00012);
        //backArm.setPIDFCoefficients(0.016, 0, 0.0003, 0);

        backSlides.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        slidesUpBasket = frontSlides.getExtensionAction(1500);
        slidesDownBasket = frontSlides.getExtensionAction(0);

        armDownSample = new InstantAction(() -> {
            frontArm.setPosition(HardwareConstants.PITCH_PICKUP);
        });
        armUpSample = new InstantAction(() -> {
            frontArm.setPosition(HardwareConstants.PITCH_ZERO);
        });
        armAboveSample = new InstantAction(() -> {
            frontArm.setPosition(HardwareConstants.PITCH_HOVER);
        });
        subPitch = new InstantAction(() -> {
            frontArm.setPosition(HardwareConstants.PITCH_HOVER - 0.1);
        });;

        parkSlidesUp = backSlides.getExtensionAction(580);
        //parkPitch = backArm.getPitchingAction(300);

        parkPitch = new InstantAction(() -> {
            pitchBack1.setPosition(0.15);
            pitchBack2.setPosition(0.15);
        });


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(imuParams);


        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.85);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);

        rightExt = hardwareMap.get(ServoImplEx.class, "rightExt");
        rightExt.setPwmRange(new PwmControl.PwmRange(500, 2500));
        leftExt = hardwareMap.get(ServoImplEx.class, "leftExt");
        leftExt.setPwmRange(new PwmControl.PwmRange(500, 2500));

        wristFront = hardwareMap.get(Servo.class, "frontWrist");

        extensionOut = new InstantAction(() -> {
            leftExt.setPosition(0.25);
            rightExt.setPosition(0.25);
        });

        extensionIn = new InstantAction(() -> {
            leftExt.setPosition(1);
            rightExt.setPosition(1);
        });

        wristDropoff = new InstantAction(() -> {
            wristFront.setPosition(1);
        });

        wristPickup = new InstantAction(() -> {
            wristFront.setPosition(0.5);
        });

        wristPickupRot = new InstantAction(() -> {
            wristFront.setPosition(0.75);
        });

        imuReset = new InstantAction(() -> {
            imu.resetYaw();
        });

        slowFollower = new InstantAction(() -> {
            follower.setMaxPower(0.2);
        });

        quickFollower = new InstantAction(() -> {
            follower.setMaxPower(1);
        });

        medFollower = new InstantAction(() -> {
            follower.setMaxPower(0.6);
        });

        EventAction subWrist = new InstantAction(() -> {
            wristFront.setPosition(wristOffset);
        });


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(8.625, 105.125, Math.toRadians(90)));

        perFollower = new PeriodicFollower(follower);

        //=================PATH CREATION==================

        Point SAMPLE_1_PICKUP = MathFunctions.addPoints(AutoConstants.NEUTRAL_SAMPLE_1, new Point(-26,0));
        Point SAMPLE_2_PICKUP = MathFunctions.addPoints(AutoConstants.NEUTRAL_SAMPLE_2, new Point(-26, -2));
        Point SAMPLE_3_PICKUP = MathFunctions.addPoints(AutoConstants.NEUTRAL_SAMPLE_3, new Point(-20, -18));


        basketPath1 = new Path(new BezierLine(
                new Point(8.625, 105.125),
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(0, 0))
        ));
        basketPath1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135));

        samplePath2 = new Path(new BezierLine(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(0, 0)),
                SAMPLE_1_PICKUP
        ));
        samplePath2.setLinearHeadingInterpolation(Math.toRadians(135), 0);

        basketPath2 = new Path(new BezierLine(
                SAMPLE_1_PICKUP,
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(1.5, 1))
        ));
        basketPath2.setLinearHeadingInterpolation(0, Math.toRadians(135));

        samplePath3 = new Path(new BezierLine(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(1.5, 1)),
                SAMPLE_2_PICKUP
        ));
        samplePath3.setLinearHeadingInterpolation(Math.toRadians(135), 0);

        basketPath3 = new Path(new BezierLine(
                SAMPLE_2_PICKUP,
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(1.5, 1.5))
        ));
        basketPath3.setLinearHeadingInterpolation(0, Math.toRadians(135));

        samplePath4 = new Path(new BezierLine(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(1.5, 1.5)),
                SAMPLE_3_PICKUP
        ));
        samplePath4.setLinearHeadingInterpolation(0, Math.toRadians(45));

        awaySamplePath4 = new Path(new BezierLine(
                SAMPLE_3_PICKUP,
                MathFunctions.addPoints(SAMPLE_3_PICKUP, new Point(0,-5))
        ));
        awaySamplePath4.setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90));

        basketPath4 = new Path(new BezierLine(
                SAMPLE_3_PICKUP,
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(2.5, 2))
        ));
        basketPath4.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135));

        parkPath = new Path(new BezierCurve(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(2.5, 2)),
                new Point(64, 130),
                new Point(62, 94)
        ));
        parkPath.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270), 0.3);
        parkPath.setZeroPowerAccelerationMultiplier(2);


        //==========PEDRO ACTION CREATION================


        moveBasket1 = new PedroAction(follower, new PathChain(basketPath1), true);

        moveSample2 = new PedroAction(follower, new PathChain(samplePath2), true);
        moveBasket2 = new PedroAction(follower, new PathChain(basketPath2), true);

        moveSample3 = new PedroAction(follower, new PathChain(samplePath3), true);
        moveBasket3 = new PedroAction(follower, new PathChain(basketPath3), true);

        moveSample4 = new PedroAction(follower, new PathChain(samplePath4), true);
        moveAwaySample4 = new PedroAction(follower, new PathChain(awaySamplePath4), true);
        moveBasket4 = new PedroAction(follower, new PathChain(basketPath4), true);

        park = new PedroAction(follower, new PathChain(parkPath), true);

        //=========================================

        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(perFollower, frontSlides, backSlides);

        wristFront.setPosition(0);
        leftExt.setPosition(1);
        rightExt.setPosition(1);


//        Gamepad prevGamepad2 = new Gamepad();
//        Gamepad currentGamepad2 = new Gamepad();

        while (!isStarted()) {

//            prevGamepad2.copy(currentGamepad2);
//            currentGamepad2.copy(gamepad2);
//
//            if (gamepad2.dpad_left && !prevGamepad2.dpad_left) {
//                subXOffset += 0.5;
//            } else if (gamepad2.dpad_up && !prevGamepad2.dpad_up) {
//                subYOffset += 0.5;
//            }
//
//            if (gamepad2.left_bumper && !prevGamepad2.left_bumper) {
//                wristOffset -= 0.1;
//            } else if (gamepad2.right_bumper && !prevGamepad2.right_bumper) {
//                wristOffset += 0.1;
//            }
//
//            if (gamepad2.options) {
//                subPath1 = new Path(new BezierCurve(
//                        MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(0.5, 1.5)),
//                        new Point(64, 130),
//                        new Point(48 + subXOffset, 86 - subYOffset + HardwareConstants.MAX_EXTENDO_LENGTH)
//                ));
//                subPath1.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270), 0.3);
//
//                basketPath5 = new Path(new BezierCurve(
//                        new Point(48 + subXOffset, 86 - subYOffset + HardwareConstants.MAX_EXTENDO_LENGTH),
//                        new Point(64, 130),
//                        MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(0.5, 1.5))
//                ));
//                basketPath5.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135), 0.8);
//            }
//
//            if (subPath1 != null) {
//                telemetry.addData("Status", "Ready to Run");
//            } else {
//                telemetry.addData("Status", "Initialize Sub Motion (Options)");
//            }
//            telemetry.addData("X Offset", subXOffset);
//            telemetry.addData("Y Offset", subYOffset);
//            telemetry.addData("Wrist Offset", wristOffset);


            telemetry.addData("-----", "");
            telemetry.update();

        }

//        if (subPath1 == null) {
//            subPath1 = new Path(new BezierCurve(
//                    MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(0.5, 1.5)),
//                    new Point(64, 130),
//                    new Point(48 + 14, 86 - 5 + HardwareConstants.MAX_EXTENDO_LENGTH)
//            ));
//            subPath1.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270), 0.3);
//
//            basketPath5 = new Path(new BezierCurve(
//                    new Point(48 + 14, 86 - 5 + HardwareConstants.MAX_EXTENDO_LENGTH),
//                    new Point(64, 130),
//                    MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(0.5, 1.5))
//            ));
//            basketPath5.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135), 0.8);
//
//        }

        if (isStopRequested()) {
            return;
        }

        actionManager.runActionManager(

                new SequentialAction(

                        //dropoff first sample

                        new ParallelAction(moveBasket1, slidesUpBasket,
                                extensionOut, wristDropoff
                        ),
                        new TimedAction(clawOuttake, 500),

                        //pickup second sample

                        new ParallelAction(moveSample2, slidesDownBasket, wristPickup),
                        imuReset,
                        new EndAction(new SequentialAction(armAboveSample, new WaitAction(1000), armDownSample, new WaitAction(1000)),
                                clawIntake
                        ),
                        armUpSample,

                        //dropoff second sample

                        new ParallelAction(moveBasket2, slidesUpBasket, wristDropoff),
                        new TimedAction(clawOuttake, 500),

                        //pickup third sample

                        new ParallelAction(moveSample3, slidesDownBasket, wristPickup),
                        new EndAction(new SequentialAction(armAboveSample, new WaitAction(1000), armDownSample, new WaitAction(1000)),
                                clawIntake
                        ),
                        armUpSample,

                        //dropoff third sample

                        new ParallelAction(moveBasket3, slidesUpBasket, wristDropoff),
                        new TimedAction(clawOuttake, 500),

                        //pickup fourth sample

                        new ParallelAction(moveSample4, slidesDownBasket, wristPickupRot),
                        new EndAction(new SequentialAction(armAboveSample, new WaitAction(1000), armDownSample, new WaitAction(1000)),
                                clawIntake
                        ),
                        new ParallelAction(moveAwaySample4, armUpSample),

                        //dropoff fourth sample

                        new ParallelAction(moveBasket4, slidesUpBasket, wristDropoff),
                        new TimedAction(clawOuttake, 500),

//                        //get fifth sample
//
//                        new ParallelAction(
//                                new PedroAction(follower, new PathChain(subPath1), true),
//                                slidesDownBasket,
//                                new SequentialAction(new WaitAction(2500), new ParallelAction(extensionOut, subPitch, subWrist))
//                        ),
//                        new WaitAction(200),
//                        new EndAction(
//                                new SequentialAction(armDownSample, new WaitAction(700)),
//                                clawIntake
//                        ),
//
//                        //dropoff fifth sample
//
//                        new ParallelAction(extensionIn, armUpSample),
//                        new ParallelAction(
//                                new PedroAction(follower, new PathChain(basketPath5), true),
//                                wristDropoff,
//                                extensionOut,
//                                new SequentialAction(new WaitAction(1000), slidesUpBasket)
//                        ),
//                        new TimedAction(clawOuttake, 500),

                        //go park

                        new ParallelAction(
                                park,
                                new SequentialAction(
                                        slidesDownBasket,
//                                        new ParallelAction(
//                                                parkSlidesUp,
//                                                parkPitch
//                                        ),
                                extensionIn, wristPickupRot),
                                new SequentialAction(new WaitAction(2000), slowFollower)
                        ),
                        new WaitAction(30000)



                ), (telemetry) -> {}

        );
    }
}

