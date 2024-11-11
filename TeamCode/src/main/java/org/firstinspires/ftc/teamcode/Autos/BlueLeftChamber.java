package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.FailsafeAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.FailsafeTrigger;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PositionFailsafe;
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.P2PAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.PositionLockAction;
import org.firstinspires.ftc.teamcode.ActionUtils.TimedAction;
import org.firstinspires.ftc.teamcode.ActionUtils.WaitAction;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.PitchArm;


@Autonomous(name = "BlueLeftChamber")
public class BlueLeftChamber extends LinearOpMode {

    ActionManager actionManager;
    MecanumPowerDrive drive;

    P2PAction moveChamber, moveChamberAway, moveSample1, moveSample2, moveSample3, moveBucket, moveAwayBucket, moveToPark, moveToPark2;

    OuttakeSlides frontSlides;
    PitchArm frontArm;

    EventAction slidesUpChamber, slidesDownChamber, slidesScoreChamber, slidesUpBucket, slidesDownBucket, armUpChamber, armDownChamber, armDownSample, armUpSample, slidesUpPark;
    ClawRotateAction clawOuttake, clawIntake;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, 15.375, 0), telemetry);
        drive.imu.resetYaw();

        moveChamber = new P2PAction(drive, new Pose2d(-35,0,0), 5, 5);
        moveChamberAway = new P2PAction(drive, new Pose2d(-40,0, 0), 5, 5);

        moveSample1 = new P2PAction(drive, new Pose2d(-45.5, 50, 0), 5, 5);
        moveSample2 = new P2PAction(drive, new Pose2d(-45.5, 60.5, 0), 5, 5);
        moveSample3 = new P2PAction(drive, new Pose2d(-42.9, 60.3, Math.toRadians(30)), 5, 5);

        moveBucket = new P2PAction(drive, new Pose2d(-57.5, 57.5, Math.toRadians(125)), 5, 5);
        moveAwayBucket = new P2PAction(drive, new Pose2d(-53, 53, Math.toRadians(125)), 5, 5);

        moveToPark = new P2PAction(drive, new Pose2d(-14, 40, Math.toRadians(-90)), 5, 5);
        moveToPark2 = new P2PAction(drive, new Pose2d(-14, 26, Math.toRadians(-90)), 5, 5);


        frontSlides = new OuttakeSlides(hardwareMap, "frontSlide");
        frontArm = new PitchArm(hardwareMap, "frontPitch");

        slidesUpChamber = frontSlides.getExtensionAction(1100);
        slidesScoreChamber = frontSlides.getExtensionAction(400);
        slidesDownChamber = frontSlides.getExtensionAction(0);
        slidesUpBucket = frontSlides.getExtensionAction(2550);
        slidesDownBucket = frontSlides.getExtensionAction(0);
        slidesUpPark = frontSlides.getExtensionAction(1000);

        //fix these
        armDownChamber = frontArm.getPitchingAction(0);
        armUpChamber = frontArm.getPitchingAction(150);
        armDownSample = frontArm.getPitchingAction(1010);
        armUpSample = frontArm.getPitchingAction(0);

        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.5);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);
        //clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.3);


        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(drive, frontSlides, frontArm);


        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(

                new SequentialAction(

                        new ParallelAction(moveChamber, slidesUpChamber),
                        armUpChamber,
                        new ParallelAction(slidesScoreChamber, new TimedAction(clawOuttake, 600)),
                        new ParallelAction(moveChamberAway, slidesDownChamber, armDownChamber),


                        moveSample1,
                        new ParallelAction(armDownSample, new TimedAction(clawIntake, 1000)),
                        armUpSample,
                        moveBucket,
                        slidesUpBucket,
                        new TimedAction(clawOuttake, 1000),
                        slidesDownBucket,

                        moveSample2,
                        new ParallelAction(armDownSample, new TimedAction(clawIntake, 1000)),
                        armUpSample,
                        moveBucket,
                        slidesUpBucket,
                        new TimedAction(clawOuttake, 1000),
                        slidesDownBucket,

                        moveSample3,
                        new ParallelAction(armDownSample, new TimedAction(clawIntake, 1000)),
                        armUpSample,
                        moveBucket,
                        slidesUpBucket,
                        new TimedAction(clawOuttake, 1000),
                        slidesDownBucket,

                        moveToPark,
                        moveToPark2,
                        slidesUpPark

                )
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }

}
