package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.Hardware.Chassis.PeriodicFollower;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;
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

    OuttakeSlides frontSlides, backSlides;
    PitchArm frontArm, backArm;
    ServoImplEx rightExt, leftExt;

    Servo wristFront;

    IMU imu;

    EventAction slidesUpBasket, slidesDownBasket,
            armDownSample, armAboveSample, armUpSample,
            extensionOut, extensionIn,
            imuReset,
            slowFollower, quickFollower, medFollower,
            wristDropoff, wristPickup, wristPickupRot,
            parkSlidesUp, parkPitch;

    ClawRotateAction clawOuttake, clawIntake;

    PedroAction moveBasket1, moveSample2, moveBasket2,
            moveSample3, moveBasket3, moveSample4, moveAwaySample4, moveBasket4,
            park;

    Path basketPath1, samplePath2, basketPath2, samplePath3, basketPath3,
            samplePath4, awaySamplePath4, basketPath4,
            parkPath;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        frontSlides = new OuttakeSlides(hardwareMap, "frontSlide");
        frontArm = new PitchArm(hardwareMap, "frontPitch");
        backSlides = new OuttakeSlides(hardwareMap, "backSlide");
        backArm = new PitchArm(hardwareMap, "backPitch");

        frontSlides.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlides.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontArm.pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontArm.pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backSlides.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlides.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backArm.pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backArm.pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontArm.setPIDFCoefficients(0.035, 0, 0.0005, 0);
        frontSlides.setPIDFCoefficients(0.026, 0, 0.00026, 0.00012);
        //backSlides.setPIDFCoefficients(0.08, 0, 0.00026, 0.00012);
        backArm.setPIDFCoefficients(0.016, 0, 0.0003, 0);

        frontSlides.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        slidesUpBasket = frontSlides.getExtensionAction(2400);
        slidesDownBasket = frontSlides.getExtensionAction(0);

        armDownSample = frontArm.getPitchingAction(1020);
        armUpSample = frontArm.getPitchingAction(0);
        armAboveSample = frontArm.getPitchingAction(780);

        parkSlidesUp = backSlides.getExtensionAction(1800);
        parkPitch = backArm.getPitchingAction(300);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(imuParams);


        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -1);
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
            wristFront.setPosition(0);
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


        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(8.625, 105.125, Math.toRadians(90)));

        perFollower = new PeriodicFollower(follower);

        //=================PATH CREATION==================

        Point SAMPLE_1_PICKUP = MathFunctions.addPoints(AutoConstants.NEUTRAL_SAMPLE_1, new Point(-26,0));
        Point SAMPLE_2_PICKUP = MathFunctions.addPoints(AutoConstants.NEUTRAL_SAMPLE_2, new Point(-26, -2));
        Point SAMPLE_3_PICKUP = MathFunctions.addPoints(AutoConstants.NEUTRAL_SAMPLE_3, new Point(-20, -18));


        basketPath1 = new Path(new BezierLine(
                new Point(8.625, 105.125),
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-3, 2))
        ));
        basketPath1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135));

        samplePath2 = new Path(new BezierLine(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-3, 2)),
                SAMPLE_1_PICKUP
        ));
        samplePath2.setLinearHeadingInterpolation(Math.toRadians(135), 0);

        basketPath2 = new Path(new BezierLine(
                SAMPLE_1_PICKUP,
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-0.5, 6))
        ));
        basketPath2.setLinearHeadingInterpolation(0, Math.toRadians(135));

        samplePath3 = new Path(new BezierLine(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-0.5, 6)),
                SAMPLE_2_PICKUP
        ));
        samplePath3.setLinearHeadingInterpolation(Math.toRadians(135), 0);

        basketPath3 = new Path(new BezierLine(
                SAMPLE_2_PICKUP,
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-1, 7))
        ));
        basketPath3.setLinearHeadingInterpolation(0, Math.toRadians(135));

        samplePath4 = new Path(new BezierLine(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-1, 7)),
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
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-1, 7))
        ));
        basketPath4.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135));

        parkPath = new Path(new BezierCurve(
                MathFunctions.addPoints(AutoConstants.BASKET_DROPOFF, new Point(-1, 7)),
                new Point(64, 130),
                new Point(62, 93)
        ));
        parkPath.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90), 0.3);


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
        actionManager.attachPeriodicActions(perFollower, frontArm, frontSlides, backArm, backSlides);

        wristFront.setPosition(0);
        leftExt.setPosition(1);
        rightExt.setPosition(1);

        telemetry.addData("Status", "Ready to Run");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        actionManager.runActionManager(

                new SequentialAction(

                        //dropoff first sample

                        new ParallelAction(moveBasket1, slidesUpBasket,
                                extensionOut, wristDropoff
                        ),
                        new TimedAction(clawOuttake, 300),

                        //pickup second sample

                        new ParallelAction(moveSample2, slidesDownBasket, wristPickup),
                        imuReset,
                        new EndAction(new SequentialAction(armDownSample, new WaitAction(400)),
                                clawIntake
                        ),
                        armUpSample,

                        //dropoff second sample

                        new ParallelAction(moveBasket2, slidesUpBasket, wristDropoff),
                        new TimedAction(clawOuttake, 400),

                        //pickup third sample

                        new ParallelAction(moveSample3, slidesDownBasket, wristPickup),
                        new EndAction(new SequentialAction(armDownSample, new WaitAction(400)),
                                clawIntake
                        ),
                        armUpSample,

                        //dropoff third sample

                        new ParallelAction(moveBasket3, slidesUpBasket, wristDropoff),
                        new TimedAction(clawOuttake, 400),

                        //pickup fourth sample

                        new ParallelAction(moveSample4, slidesDownBasket, wristPickupRot),
                        new EndAction(new SequentialAction(armDownSample, new WaitAction(400)),
                                clawIntake
                        ),
                        new ParallelAction(moveAwaySample4, armUpSample),

                        //dropoff fourth sample

                        new ParallelAction(moveBasket4, slidesUpBasket, wristDropoff),
                        new TimedAction(clawOuttake, 400),

                        //go park

                        new ParallelAction(
                                park,
                                new SequentialAction(
                                        slidesDownBasket,
                                        new ParallelAction(
                                                parkSlidesUp,
                                                parkPitch
                                        ),
                                extensionIn, wristPickupRot),
                                new SequentialAction(new WaitAction(2000), slowFollower)
                        ),
                        new WaitAction(30000)



                ), (telemetry) -> {}

        );
    }
}

