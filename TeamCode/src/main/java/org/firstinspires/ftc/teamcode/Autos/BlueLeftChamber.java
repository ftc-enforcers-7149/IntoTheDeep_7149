package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.InstantAction;
import org.firstinspires.ftc.teamcode.ActionUtils.WristRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.P2PAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TimedAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.WaitAction;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;

@Disabled
@Autonomous(name = "BlueLeftChamber")
public class BlueLeftChamber extends LinearOpMode {

    ActionManager actionManager;
    MecanumPowerDrive drive;

    ServoImplEx rightExt, leftExt;

    P2PAction moveChamber, moveChamberAway, moveSample1, moveSample2, moveSample3, moveBucket, moveAwayBucket, moveToPark, moveToPark2;

    OuttakeSlides frontSlides;
    PitchArm frontArm;
    Servo wristFront;

    WristRotateAction sampleRotate, wristReset;

    EventAction slidesUpChamber, slidesDownChamber, slidesScoreChamber,
            slidesUpBucket, slidesDownBucket, armUpChamber, armDownChamber,
            armDownSample, armUpSample, slidesUpPark, armPark,
            extensionOut, extensionIn;
    ClawRotateAction clawOuttake, clawIntake;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, 15.375, 0), telemetry);
        drive.imu.resetYaw();

        moveChamber = new P2PAction(drive, new Pose2d(-35.5,0,0), 5, 5);
        moveChamberAway = new P2PAction(drive, new Pose2d(-40,0, 0), 5, 5);

        moveSample1 = new P2PAction(drive, new Pose2d(-46.3, 50, 0), 5, 5);
        moveSample2 = new P2PAction(drive, new Pose2d(-45.8, 60.5, 0), 5, 5);
        moveSample3 = new P2PAction(drive, new Pose2d(-43.3, 60.3, Math.toRadians(30)), 5, 5);

        moveBucket = new P2PAction(drive, new Pose2d(-57, 57, Math.toRadians(125)), 5, 5);
        moveAwayBucket = new P2PAction(drive, new Pose2d(-53, 53, Math.toRadians(125)), 5, 5);

        moveToPark = new P2PAction(drive, new Pose2d(-12.5, 40, Math.toRadians(-90)), 5, 5);
        moveToPark2 = new P2PAction(drive, new Pose2d(-12.5, 27, Math.toRadians(-90)), 5, 5);

        frontSlides = new OuttakeSlides(hardwareMap, "frontSlide");
        frontSlides.slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontArm = new PitchArm(hardwareMap, "frontPitch");
        wristFront = hardwareMap.get(Servo.class, "frontWrist");

        rightExt = hardwareMap.get(ServoImplEx.class, "rightExt");
        rightExt.setPwmRange(new PwmControl.PwmRange(500, 2500));
        leftExt = hardwareMap.get(ServoImplEx.class, "leftExt");
        leftExt.setPwmRange(new PwmControl.PwmRange(500, 2500));

        frontSlides.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlides.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontArm.pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontArm.pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesUpChamber = frontSlides.getExtensionAction(1100);
        slidesScoreChamber = frontSlides.getExtensionAction(400);
        slidesDownChamber = frontSlides.getExtensionAction(0);
        slidesUpBucket = frontSlides.getExtensionAction(2200);
        slidesDownBucket = frontSlides.getExtensionAction(0);
        slidesUpPark = frontSlides.getExtensionAction(1000);

        //fix these
        armDownChamber = frontArm.getPitchingAction(0);
        armUpChamber = frontArm.getPitchingAction(100);
        armDownSample = frontArm.getPitchingAction(960);
        armUpSample = frontArm.getPitchingAction(0);

        armPark = frontArm.getPitchingAction(150);

        extensionOut = new InstantAction(() -> {
            leftExt.setPosition(0.25);
            rightExt.setPosition(0.25);
        });

        extensionIn = new InstantAction(() -> {
            leftExt.setPosition(1);
            rightExt.setPosition(1);
        });

        sampleRotate = new WristRotateAction(0.3, hardwareMap, "frontWrist");
        wristReset = new WristRotateAction(0.5, hardwareMap, "frontWrist");

        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.6);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);
        //clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.3);


        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(drive, frontSlides, frontArm);

        wristFront.setPosition(0.5);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(

                new SequentialAction(

                        //score specimen

                        new ParallelAction(moveChamber, slidesUpChamber),
                        armUpChamber,
                        new WaitAction(500),
                        new ParallelAction(slidesScoreChamber, new TimedAction(clawOuttake, 600)),
                        new ParallelAction(moveChamberAway, slidesDownChamber, armDownChamber),

                        //move to sample 1 and score in the bucket

                        moveSample1,
                        new ParallelAction(armDownSample, new TimedAction(clawIntake, 1000)),
                        armUpSample,
                        moveBucket,
                        new ParallelAction(slidesUpBucket, extensionOut),
                        new TimedAction(clawOuttake, 400),
                        new ParallelAction(slidesDownBucket, extensionIn),

                        //move to sample 2 and score in the bucket

                        moveSample2,
                        new ParallelAction(armDownSample, new TimedAction(clawIntake, 1000)),
                        new ParallelAction(armUpSample, extensionIn),
                        moveBucket,
                        new ParallelAction(slidesUpBucket, extensionOut),
                        new TimedAction(clawOuttake, 400),
                        new ParallelAction(slidesDownBucket, extensionIn),

                        //move to sample 3 and score in the bucket

                        moveSample3,
                        new ParallelAction(armDownSample, sampleRotate, new TimedAction(clawIntake, 1000)),
                        new ParallelAction(armUpSample, wristReset),
                        moveBucket,
                        new ParallelAction(slidesUpBucket, extensionOut),
                        new TimedAction(clawOuttake, 400),
                        new ParallelAction(slidesDownBucket, extensionIn)

                        //park

//                        moveToPark,
//                        new ParallelAction(moveToPark2, armPark, extensionOut)

                )
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }

}
