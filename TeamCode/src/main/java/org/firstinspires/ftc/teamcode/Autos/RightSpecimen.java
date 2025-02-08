package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TimedAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.WaitAction;
import org.firstinspires.ftc.teamcode.GlobalData.AutoConstants;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "5 Specimen Auto")
public class RightSpecimen extends LinearOpMode {

    ActionManager actionManager;
    Follower follower;

    OuttakeSlides frontSlides, backSlides;
    PitchArm frontArm, backArm;
    ServoImplEx rightExt, leftExt;

    Servo wristFront;

    IMU imu;

    EventAction slidesUpChamber, slidesDownChamber, slidesScoreChamber,
            slidesBackUpChamber, slidesBackAboveWall, slidesBackDownChamber, slidesBackScoreChamber,
            armUpChamber, armDownChamber, armDownSample, armAboveSample, armUpSample,
            armBackUpChamber, armBackDownChamber,
            extensionOut, extensionIn,
            imuReset,
            slowFollower, quickFollower, medFollower;

    ClawRotateAction clawOuttake, clawIntake, clawBackIntake, clawBackOuttake;

    PedroAction moveChamber1, moveSample1, depositSample1, moveSample2, depositSample2, moveSample3, depositSample3,
            moveWall2, moveChamber2, moveWall3, moveChamber3,
            moveWall4, moveChamber4, moveWall5, moveChamber5,
            park;

    Path chamberPath1, samplePath1, depositPath1, samplePath2, depositPath2, samplePath3, depositPath3,
            wallPath2, chamberPath2, wallPath3, chamberPath3,
            wallPath4, chamberPath4, wallPath5, chamberPath5,
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

        frontArm.setPIDFCoefficients(0.054, 0, 0.0005,0);
        //backSlides.setPIDFCoefficients(0.08, 0, 0.00026, 0.00012);
        backArm.setPIDFCoefficients(0.016, 0, 0.0003,0);

        frontSlides.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        slidesUpChamber = frontSlides.getExtensionAction(1500);
        slidesScoreChamber = frontSlides.getExtensionAction(550);
        slidesDownChamber = frontSlides.getExtensionAction(0);

        slidesBackUpChamber = backSlides.getExtensionAction(1800);
        slidesBackScoreChamber = backSlides.getExtensionAction(1000);
        slidesBackDownChamber = backSlides.getExtensionAction(0);
        slidesBackAboveWall = backSlides.getExtensionAction(300);

        armDownSample = frontArm.getPitchingAction(1040);
        armUpSample = frontArm.getPitchingAction(0);
        armAboveSample = frontArm.getPitchingAction(780);

        armBackDownChamber = backArm.getPitchingAction(0);
        armBackUpChamber = backArm.getPitchingAction(150);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(imuParams);


        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.6);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);

        clawBackOuttake = new ClawRotateAction(hardwareMap, "backClaw", 0.7);
        clawBackIntake = new ClawRotateAction(hardwareMap, "backClaw", 1);

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

        imuReset = new InstantAction(() -> {
            imu.resetYaw();
        });

        slowFollower = new InstantAction(() -> {
            follower.setMaxPower(0.4);
        });

        quickFollower = new InstantAction(() -> {
            follower.setMaxPower(1);
        });

        medFollower = new InstantAction(() -> {
            follower.setMaxPower(0.6);
        });


        follower = new Follower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(8.75, 62, Math.toRadians(180)));


        //==========PATH CREATION================================

        Point SAMPLE_1_PICKUP = MathFunctions.addPoints(AutoConstants.ALLIANCE_SAMPLE_1, AutoConstants.ALLIANCE_PICKUP_OFFSET);
        Point SAMPLE_2_PICKUP = MathFunctions.addPoints(AutoConstants.ALLIANCE_SAMPLE_2, AutoConstants.ALLIANCE_PICKUP_OFFSET);
        Point SAMPLE_3_PICKUP = MathFunctions.addPoints(AutoConstants.ALLIANCE_SAMPLE_3, AutoConstants.ALLIANCE_PICKUP_OFFSET);


        chamberPath1 = new Path(new BezierLine(
                new Point(8.75, 62),
                new Point(30, 64)
        ));
        chamberPath1.setConstantHeadingInterpolation(Math.toRadians(180));


        samplePath1 = new Path(new BezierCurve(
                new Point(30, 64),
                new Point(30, 54),
                SAMPLE_1_PICKUP
        ));
        samplePath1.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45));

        depositPath1 = new Path(new BezierLine(
                SAMPLE_1_PICKUP,
                MathFunctions.addPoints(SAMPLE_1_PICKUP, new Point(-2, -4))
        ));
        depositPath1.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-135));

        samplePath2 = new Path(new BezierLine(
                MathFunctions.addPoints(SAMPLE_1_PICKUP, new Point(-2, -4)),
                SAMPLE_2_PICKUP
        ));
        samplePath2.setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-45));

        depositPath2 = new Path(new BezierLine(
                SAMPLE_2_PICKUP,
                MathFunctions.addPoints(SAMPLE_2_PICKUP, new Point(-2, -4))
        ));
        depositPath2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-135));

        samplePath3 = new Path(new BezierCurve(
                MathFunctions.addPoints(SAMPLE_2_PICKUP, new Point(-2, -4)),
                new Point(24,38),
                SAMPLE_3_PICKUP
        ));
        samplePath3.setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-45));

        depositPath3 = new Path(new BezierLine(
                SAMPLE_3_PICKUP,
                MathFunctions.addPoints(SAMPLE_3_PICKUP, new Point(-2, 7))
        ));
        depositPath3.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-135));

        //---------------

        wallPath2 = new Path(new BezierCurve(
                MathFunctions.addPoints(SAMPLE_3_PICKUP, new Point(-2, 7)),
                new Point(36, 36),
                MathFunctions.addPoints(AutoConstants.SPECIMEN_PICKUP, new Point(-2.5, 0))
        ));
        wallPath2.setLinearHeadingInterpolation(Math.toRadians(-135), 0, 0.2);
        wallPath2.setZeroPowerAccelerationMultiplier(6);

        chamberPath2 = new Path(new BezierLine(
                MathFunctions.addPoints(AutoConstants.SPECIMEN_PICKUP, new Point(-2.5, 0)),
                new Point(36, 61)
        ));
        chamberPath2.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.6);

        wallPath3 = new Path(new BezierCurve(
                new Point(36, 61),
                new Point(30, 40),
                AutoConstants.SPECIMEN_PICKUP
        ));
        wallPath3.setLinearHeadingInterpolation(Math.toRadians(180.01), 0, 0.2);
        wallPath3.setZeroPowerAccelerationMultiplier(6);

        chamberPath3 = new Path(new BezierLine(
                AutoConstants.SPECIMEN_PICKUP,
                new Point(36, 62)
        ));
        chamberPath3.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.6);

        wallPath4 = new Path(new BezierCurve(
                new Point(36, 62),
                new Point(30, 40),
                AutoConstants.SPECIMEN_PICKUP
        ));
        wallPath4.setLinearHeadingInterpolation(Math.toRadians(180.01), 0, 0.2);
        wallPath4.setZeroPowerAccelerationMultiplier(6);

        chamberPath4 = new Path(new BezierLine(
                AutoConstants.SPECIMEN_PICKUP,
                new Point(36, 66)
        ));
        chamberPath4.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.6);

        wallPath5 = new Path(new BezierCurve(
                new Point(36, 66),
                new Point(30, 40),
                AutoConstants.SPECIMEN_PICKUP
        ));
        wallPath5.setLinearHeadingInterpolation(Math.toRadians(180.01), 0, 0.2);
        wallPath5.setZeroPowerAccelerationMultiplier(6);

        chamberPath5 = new Path(new BezierLine(
                AutoConstants.SPECIMEN_PICKUP,
                new Point(36, 68)
        ));
        chamberPath5.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.6);

        parkPath = new Path(new BezierLine(
                new Point(36, 68),
                new Point(24, 36)
        ));
        parkPath.setLinearHeadingInterpolation(Math.toRadians(180.01), Math.toRadians(215), 0.3);


        //==========PEDRO ACTION CREATION====================


        moveChamber1 = new PedroAction(follower, new PathChain(chamberPath1), true);

        moveSample1 = new PedroAction(follower, new PathChain(samplePath1), true);
        depositSample1 = new PedroAction(follower, new PathChain(depositPath1), true);

        moveSample2 = new PedroAction(follower, new PathChain(samplePath2), true);
        depositSample2 = new PedroAction(follower, new PathChain(depositPath2), true);

        moveSample2 = new PedroAction(follower, new PathChain(samplePath2), true);
        depositSample2 = new PedroAction(follower, new PathChain(depositPath2), true);

        moveSample3 = new PedroAction(follower, new PathChain(samplePath3), true);
        depositSample3 = new PedroAction(follower, new PathChain(depositPath3), true);

        moveWall2 = new PedroAction(follower, new PathChain(wallPath2), true);
        moveChamber2 = new PedroAction(follower, new PathChain(chamberPath2), true);

        moveWall3 = new PedroAction(follower, new PathChain(wallPath3), true);
        moveChamber3 = new PedroAction(follower, new PathChain(chamberPath3), true);

        moveWall4 = new PedroAction(follower, new PathChain(wallPath4), true);
        moveChamber4 = new PedroAction(follower, new PathChain(chamberPath4), true);

        moveWall5 = new PedroAction(follower, new PathChain(wallPath5), true);
        moveChamber5 = new PedroAction(follower, new PathChain(chamberPath5), true);

        park = new PedroAction(follower, new PathChain(parkPath), true);

        //===============================================


        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(follower, frontArm, backSlides, backArm);

        wristFront.setPosition(0.25);
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

                        //score first spec on chamber

                        new ParallelAction(medFollower,
                                new SequentialAction(new WaitAction(200),moveChamber1),
                                slidesBackUpChamber,
                                armBackUpChamber, extensionOut
                        ),
                        quickFollower,
                        new ParallelAction(
                                //new SequentialAction(new WaitAction(500), slowFollower),
                                moveSample1, slidesBackDownChamber,
                                armBackDownChamber,
                                armAboveSample,
                                new TimedAction(clawBackOuttake, 300)
                        ),

                        //pickup the first sample and deposit it

                        new EndAction(new SequentialAction(armDownSample, new WaitAction(400)), clawIntake),
                        new ParallelAction(/*quickFollower,*/ depositSample1, armAboveSample),
                        new TimedAction(clawOuttake, 250),

                        //pickup second sample and deposit it

                        //slowFollower,
                        moveSample2,
                        new EndAction(new SequentialAction(armDownSample, new WaitAction(300)), clawIntake),
                        new ParallelAction(/*quickFollower,*/ depositSample2, armAboveSample),
                        new TimedAction(clawOuttake, 250),

                        //pickup third sample and deposit it

                        //slowFollower,
                        moveSample3,
                        new EndAction(new SequentialAction(armDownSample, new WaitAction(300)), clawIntake),
                        new ParallelAction(/*quickFollower,*/ depositSample3, armAboveSample),
                        new TimedAction(clawOuttake, 250),

                        //get spec 2 off the wall and score it
                        //stop intaking 200ms after arriving at wall

                        new ParallelAction(
                                new EndAction(new SequentialAction(moveWall2, new WaitAction(300)),
                                        clawBackIntake),
                                extensionIn,
                                armUpSample
                                //new SequentialAction(new WaitAction(200), slowFollower)
                        ),
                        imuReset,
                        new ParallelAction(quickFollower, new TimedAction(clawBackOuttake, 200), moveChamber2, slidesBackUpChamber),
                        slidesBackScoreChamber,

                        //return to wall, get spec 3, score it
                        //while moving away, outtake spec, then start intaking
                        //stop intaking 200ms after arriving at wall

                        new ParallelAction(
                                new EndAction(
                                    new SequentialAction(moveWall3, new WaitAction(300)),
                                    new SequentialAction(new TimedAction(clawBackOuttake, 300),
                                        clawBackIntake)),
                                slidesBackDownChamber
                                //new SequentialAction(new WaitAction(800), slowFollower)
                        ),
                        new ParallelAction(quickFollower, new TimedAction(clawBackOuttake, 200), moveChamber3, slidesBackUpChamber),
                        slidesBackScoreChamber,

                        //return to wall, get spec 4, score it

                        new ParallelAction(
                                new EndAction(
                                        new SequentialAction(moveWall4, new WaitAction(300)),
                                        new SequentialAction(new TimedAction(clawBackOuttake, 300),
                                                clawBackIntake)),
                                slidesBackDownChamber
                                //new SequentialAction(new WaitAction(800), slowFollower)
                        ),
                        new ParallelAction(quickFollower, new TimedAction(clawBackOuttake, 200), moveChamber4, slidesBackUpChamber),
                        slidesBackScoreChamber,

                        //return to wall, get spec 5, score it

                        new ParallelAction(
                                new EndAction(
                                        new SequentialAction(moveWall5, new WaitAction(300)),
                                        new SequentialAction(new TimedAction(clawBackOuttake, 300),
                                                clawBackIntake)),
                                slidesBackDownChamber
                                //new SequentialAction(new WaitAction(800), slowFollower)
                        ),
                        new ParallelAction(quickFollower, new TimedAction(clawBackOuttake, 200), moveChamber5, slidesBackUpChamber),
                        slidesBackScoreChamber,

                        //go to park

                        new ParallelAction(park, slidesBackDownChamber)


                ), (telemetry) -> {}
        );



        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
