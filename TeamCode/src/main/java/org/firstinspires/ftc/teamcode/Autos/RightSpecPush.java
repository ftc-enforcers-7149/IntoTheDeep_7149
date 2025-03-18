package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
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
import org.firstinspires.ftc.teamcode.GlobalData.HardwareConstants;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.PeriodicFollower;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.FourServoPitchArm;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.TwoMotorSlides;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous(name = "5 Spec - Push")
public class RightSpecPush extends LinearOpMode {

    ActionManager actionManager;
    Follower follower;

    OuttakeSlides backSlides;
    TwoMotorSlides frontSlides;
    FourServoPitchArm frontArm;
    ServoImplEx rightExt, leftExt;
    Servo pitchBack1, pitchBack2;

    Servo wristFront;

    IMU imu;

    EventAction slidesUpChamber, slidesDownChamber, slidesScoreChamber,
            slidesBackUpChamber, slidesBackAboveWall, slidesBackDownChamber, slidesBackScoreChamber,
            armUpChamber, armDownChamber, armDownSample, armAboveSample, armUpSample,
            armBackUpChamber, armBackDownChamber, armBackSpecPickup, armBackSpecScore,
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


        frontSlides = new TwoMotorSlides(this);
        frontArm = new FourServoPitchArm(this);
        frontArm.setPosition(HardwareConstants.PITCH_ZERO);

        backSlides = new OuttakeSlides(hardwareMap, "backSlide");
        pitchBack1 = hardwareMap.get(Servo.class, "pitchBack1");
        pitchBack2 = hardwareMap.get(Servo.class, "pitchBack2");
        pitchBack1.setPosition(HardwareConstants.BACK_PITCH_ZERO);
        pitchBack2.setPosition(HardwareConstants.BACK_PITCH_ZERO);

        frontSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backSlides.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlides.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backSlides.setPIDFCoefficients(0.022, 0, 0.0006, 0.00022);
        backSlides.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        slidesUpChamber = frontSlides.getExtensionAction(550);
        slidesScoreChamber = frontSlides.getExtensionAction(450);
        slidesDownChamber = frontSlides.getExtensionAction(0);

        slidesBackUpChamber = backSlides.getExtensionAction(610);
        slidesBackScoreChamber = backSlides.getExtensionAction(410);
        slidesBackDownChamber = backSlides.getExtensionAction(0);
        slidesBackAboveWall = backSlides.getExtensionAction(150);

        armDownSample = new InstantAction(() -> {
            frontArm.setPosition(HardwareConstants.PITCH_HOVER);
        });
//        armUpSample = frontArm.getPitchingAction(0);
//        armAboveSample = frontArm.getPitchingAction(780);
        armUpChamber = new InstantAction(() -> {
            frontArm.setPosition(0.33);
        });
        armDownChamber = new InstantAction(() -> {
            frontArm.setPosition(HardwareConstants.PITCH_ZERO);
        });

        armBackDownChamber = new InstantAction(() -> {
            pitchBack1.setPosition(HardwareConstants.BACK_PITCH_ZERO);
            pitchBack2.setPosition(HardwareConstants.BACK_PITCH_ZERO);
        });
        armBackUpChamber = new InstantAction(() -> {
            pitchBack1.setPosition(HardwareConstants.BACK_PITCH_WALL + 0.15);
            pitchBack2.setPosition(HardwareConstants.BACK_PITCH_WALL + 0.15);
        });
        armBackSpecPickup = new InstantAction(() -> {
            pitchBack1.setPosition(HardwareConstants.BACK_PITCH_WALL);
            pitchBack2.setPosition(HardwareConstants.BACK_PITCH_WALL);
        });
        armBackSpecScore = new InstantAction(() -> {
            pitchBack1.setPosition(HardwareConstants.BACK_PITCH_SCORE);
            pitchBack2.setPosition(HardwareConstants.BACK_PITCH_SCORE);
        });

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(imuParams);


        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.6);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);

        clawBackOuttake = new ClawRotateAction(hardwareMap, "backClaw", -0.7);
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(8.75, 62, 0));

        PeriodicFollower perFollower = new PeriodicFollower(follower);
        imu.resetYaw();  //facing forwards, so reset the imu


        //==========PATH CREATION================================

        Point SAMPLE_1_PICKUP = MathFunctions.addPoints(AutoConstants.ALLIANCE_SAMPLE_1, AutoConstants.ALLIANCE_PICKUP_OFFSET);
        Point SAMPLE_2_PICKUP = MathFunctions.addPoints(AutoConstants.ALLIANCE_SAMPLE_2, AutoConstants.ALLIANCE_PICKUP_OFFSET);
        Point SAMPLE_3_PICKUP = MathFunctions.addPoints(AutoConstants.ALLIANCE_SAMPLE_3, AutoConstants.ALLIANCE_PICKUP_OFFSET);
        Point SPEC_PICKUP = MathFunctions.addPoints(AutoConstants.SPECIMEN_PICKUP, new Point(1.5, 0));

        chamberPath1 = new Path(new BezierLine(
                new Point(8.75, 62),
                new Point(30.5, 60.5)
        ));
        chamberPath1.setConstantHeadingInterpolation(Math.toRadians(0));
        chamberPath1.setZeroPowerAccelerationMultiplier(4);

        Path samplePath1_1 = new Path(new BezierCurve(
                new Point(30.5, 60.5),
                new Point(18, 35),
                new Point(46.5, 35.5)
        ));
        samplePath1_1.setConstantHeadingInterpolation(Math.toRadians(0));
        samplePath1_1.setZeroPowerAccelerationMultiplier(6);

        Path samplePath1_2 = new Path(new BezierCurve(
                new Point(33.5, 64),
                new Point(58, 40.3),
                new Point(54.5, 31.3),
                new Point(55, 29)
        ));
        samplePath1_2.setConstantHeadingInterpolation(Math.toRadians(0));
        samplePath1_2.setZeroPowerAccelerationMultiplier(6);

        Path samplePath1_3 = new Path(new BezierLine(
                new Point(55, 29),
                new Point(24, 29)
        ));
        samplePath1_3.setConstantHeadingInterpolation(Math.toRadians(0));
        samplePath1_3.setZeroPowerAccelerationMultiplier(6);

        Path samplePath2_1 = new Path(new BezierCurve(
                new Point(24, 29),
                new Point(56, 30),
                new Point(55, 18)
        ));
        samplePath2_1.setConstantHeadingInterpolation(Math.toRadians(0));
        samplePath2_1.setZeroPowerAccelerationMultiplier(6);

        Path samplePath2_2 = new Path(new BezierLine(
                new Point(55, 18),
                new Point(24, 18)
        ));
        samplePath2_2.setConstantHeadingInterpolation(Math.toRadians(0));
        samplePath2_2.setZeroPowerAccelerationMultiplier(6);

        Path samplePath3_1 = new Path(new BezierCurve(
                new Point(24, 18),
                new Point(56, 16.5),
                new Point(55, 10)
        ));
        samplePath3_1.setConstantHeadingInterpolation(Math.toRadians(0));
        samplePath3_1.setZeroPowerAccelerationMultiplier(6);

        Path samplePath3_2 = new Path(new BezierLine(
                new Point(55, 10),
                new Point(24, 10)
        ));
        samplePath3_2.setConstantHeadingInterpolation(Math.toRadians(0));
        samplePath3_2.setZeroPowerAccelerationMultiplier(6);


        //---------------

//        wallPath2 = new Path(new BezierCurve(
//                //MathFunctions.addPoints(SAMPLE_3_PICKUP, new Point(-2, 7)),
//                new Point(24, 9.5),
//                new Point(36, 36),
//                MathFunctions.addPoints(SPEC_PICKUP, new Point(0, 1))
//                //SPEC_PICKUP
//        ));
        wallPath2 = new Path(new BezierCurve(
                //MathFunctions.addPoints(SAMPLE_3_PICKUP, new Point(-2, 7)),
                new Point(24, 10.5),
                new Point(26, 11),
                new Point(11.5, 13)
        ));
        wallPath2.setConstantHeadingInterpolation(0);
        wallPath2.setZeroPowerAccelerationMultiplier(5);

        chamberPath2 = new Path(new BezierCurve(
                //SPEC_PICKUP,
                new Point(13, 13),
                new Point(30, 79),
                new Point(37, 76)
        ));
        chamberPath2.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.4);
        chamberPath2.setZeroPowerAccelerationMultiplier(1.4);

        wallPath3 = new Path(new BezierCurve(
                new Point(37, 76),
                new Point(30, 40),
                SPEC_PICKUP
        ));
        wallPath3.setLinearHeadingInterpolation(Math.toRadians(180.01), 0, 0.2);
        wallPath3.setZeroPowerAccelerationMultiplier(6);

        chamberPath3 = new Path(new BezierCurve(
                SPEC_PICKUP,
                new Point(30, 76),
                new Point(37, 73)
        ));
        chamberPath3.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.6);
        chamberPath3.setZeroPowerAccelerationMultiplier(1.4);

        wallPath4 = new Path(new BezierCurve(
                new Point(37, 73),
                new Point(30, 40),
                SPEC_PICKUP
        ));
        wallPath4.setLinearHeadingInterpolation(Math.toRadians(180.01), 0, 0.2);
        wallPath4.setZeroPowerAccelerationMultiplier(6);

        chamberPath4 = new Path(new BezierCurve(
                SPEC_PICKUP,
                new Point(30, 72),
                new Point(37, 69)
        ));
        chamberPath4.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.6);
        chamberPath4.setZeroPowerAccelerationMultiplier(1.4);

        wallPath5 = new Path(new BezierCurve(
                new Point(37, 69),
                new Point(30, 40),
                SPEC_PICKUP
        ));
        wallPath5.setLinearHeadingInterpolation(Math.toRadians(180.01), 0, 0.2);
        wallPath5.setZeroPowerAccelerationMultiplier(6);

        chamberPath5 = new Path(new BezierCurve(
                SPEC_PICKUP,
                new Point(30, 69),
                new Point(37, 66)
        ));
        chamberPath5.setLinearHeadingInterpolation(0, Math.toRadians(180.01), 0.6);
        chamberPath5.setZeroPowerAccelerationMultiplier(1.4);

        parkPath = new Path(new BezierLine(
                new Point(37, 66),
                new Point(28, 36)
        ));
        parkPath.setLinearHeadingInterpolation(Math.toRadians(180.01), Math.toRadians(230), 0.3);


        //==========PEDRO ACTION CREATION====================


        moveChamber1 = new PedroAction(follower, new PathChain(chamberPath1), true);

        moveSample1 = new PedroAction(follower, new PathChain(samplePath1_1, samplePath1_2, samplePath1_3), true);
        //depositSample1 = new PedroAction(follower, new PathChain(depositPath1), true);

        moveSample2 = new PedroAction(follower, new PathChain(samplePath2_1, samplePath2_2), true);
        //depositSample2 = new PedroAction(follower, new PathChain(depositPath2), true);

        moveSample3 = new PedroAction(follower, new PathChain(samplePath3_1, samplePath3_2), true);
        //depositSample2 = new PedroAction(follower, new PathChain(depositPath2), true);

        //moveSample3 = new PedroAction(follower, new PathChain(samplePath3), true);
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
        actionManager.attachPeriodicActions(perFollower, backSlides, frontSlides);

        wristFront.setPosition(0.48);
        leftExt.setPosition(1);
        rightExt.setPosition(1);


        while (!isStarted()) {
            if (gamepad1.triangle) {
                telemetry.addData("Pressing", "Tri");
            }

            telemetry.addData("Status", "Ready to Run");
            telemetry.update();
        }


        if (isStopRequested()) {
            return;
        }


        actionManager.runActionManager(

                new SequentialAction(

                        //score first spec on chamber

                        new ParallelAction(
                                moveChamber1,
                                //slidesBackUpChamber,
                                //armBackUpChamber,
                                extensionOut,
                                slidesUpChamber,
                                armUpChamber

                        ),
                        //quickFollower,

                        new ParallelAction(
                                //new SequentialAction(new WaitAction(500), slowFollower),
                                //slidesBackDownChamber,
                                //armBackDownChamber,
                                //armAboveSample,
                                //new TimedAction(clawBackOuttake, 300)
                                slidesDownChamber,
                                new SequentialAction(new WaitAction(1000), armDownChamber),
                                extensionIn,

                                moveSample1
                        ),

                        moveSample2,

                        moveSample3,

                        //get spec 2 off the wall and score it
                        //stop intaking 200ms after arriving at wall
                        new ParallelAction(
                                new EndAction(new SequentialAction(moveWall2, new WaitAction(100)),
                                        clawBackIntake),
                                extensionIn,
                                armBackSpecPickup
                                //new SequentialAction(new WaitAction(200), slowFollower)
                        ),
                        new TimedAction(new ParallelAction(slidesBackUpChamber, clawBackIntake), 300),
                        new ParallelAction(
                                quickFollower,
                                new TimedAction(clawBackIntake, 200),
                                moveChamber2,
                                slidesBackUpChamber,
                                new SequentialAction(new WaitAction(500), armBackDownChamber)
                        ),
                        new ParallelAction(
                                new SequentialAction(new WaitAction(150), slidesBackScoreChamber),
                                new SequentialAction(new WaitAction(300), new TimedAction(clawBackOuttake, 200)),
                                armBackSpecScore),

                        //return to wall, get spec 3, score it
                        //while moving away, outtake spec, then start intaking
                        //stop intaking 200ms after arriving at wall
                        armBackDownChamber,
                        new ParallelAction(
                                new EndAction(
                                        new SequentialAction(moveWall3, new WaitAction(100)),
                                        new SequentialAction(new TimedAction(clawBackOuttake, 300),
                                                clawBackIntake)),
                                new SequentialAction(slidesBackDownChamber, armBackSpecPickup)
                                //new SequentialAction(new WaitAction(800), slowFollower)
                        ),
                        new TimedAction(new ParallelAction(slidesBackUpChamber, clawBackIntake), 300),
                        new ParallelAction(
                                quickFollower,
                                new TimedAction(clawBackIntake, 200),
                                moveChamber3,
                                slidesBackUpChamber,
                                new SequentialAction(new WaitAction(500), armBackDownChamber)

                        ),
                        new ParallelAction(
                                new SequentialAction(new WaitAction(150), slidesBackScoreChamber),
                                new SequentialAction(new WaitAction(300), new TimedAction(clawBackOuttake, 200)),
                                armBackSpecScore),

                        //return to wall, get spec 4, score it

                        armBackDownChamber,
                        new ParallelAction(
                                new EndAction(
                                        new SequentialAction(moveWall4, new WaitAction(100)),
                                        new SequentialAction(new TimedAction(clawBackOuttake, 300),
                                                clawBackIntake)),
                                new SequentialAction(slidesBackDownChamber, armBackSpecPickup)
                                //new SequentialAction(new WaitAction(800), slowFollower)
                        ),
                        new TimedAction(new ParallelAction(slidesBackUpChamber, clawBackIntake), 300),
                        new ParallelAction(quickFollower,
                                new TimedAction(clawBackIntake, 200),
                                moveChamber4,
                                slidesBackUpChamber,
                                new SequentialAction(new WaitAction(500), armBackDownChamber)
                        ),
                        new ParallelAction(
                                new SequentialAction(new WaitAction(150), slidesBackScoreChamber),
                                new SequentialAction(new WaitAction(300), new TimedAction(clawBackOuttake, 200)),
                                armBackSpecScore),

                        //return to wall, get spec 5, score it

                        armBackDownChamber,
                        new ParallelAction(
                                new EndAction(
                                        new SequentialAction(moveWall5, new WaitAction(100)),
                                        new SequentialAction(new TimedAction(clawBackOuttake, 300),
                                                clawBackIntake)),
                                new SequentialAction(slidesBackDownChamber, armBackSpecPickup)
                                //new SequentialAction(new WaitAction(800), slowFollower)
                        ),
                        new TimedAction(new ParallelAction(slidesBackUpChamber, clawBackIntake), 300),
                        new ParallelAction(
                                quickFollower,
                                new TimedAction(clawBackIntake, 200),
                                moveChamber5,
                                slidesBackUpChamber,
                                new SequentialAction(new WaitAction(500), armBackDownChamber)
                        ),
                        new ParallelAction(
                                new SequentialAction(new WaitAction(150), slidesBackScoreChamber),
                                new SequentialAction(new WaitAction(300), new TimedAction(clawBackOuttake, 200)),
                                armBackSpecScore),

                        //go to park

                        new ParallelAction(park, slidesBackDownChamber, armDownSample)


                ), (telemetry) -> {
                    telemetry.addData("Pitch", pitchBack1.getPosition());
                    telemetry.addData("Pitch2", pitchBack2.getPosition());
                }
        );



        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
