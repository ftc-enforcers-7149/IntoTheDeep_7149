package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PositionAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes.MultiShape;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes.Rectangle;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes.Triangle;
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.EndAction;
import org.firstinspires.ftc.teamcode.ActionUtils.P2PAction;
import org.firstinspires.ftc.teamcode.ActionUtils.TimedAction;
import org.firstinspires.ftc.teamcode.ActionUtils.WaitAction;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Point;

public class RightSpecimen extends LinearOpMode {

    ActionManager actionManager;
    MecanumPowerDrive drive;

    P2PAction moveChamber, moveChamberAway, moveSample1, moveSample2, moveObservation, moveSpecimen1, moveSpecimen2, moveChamber1, moveChamber2, moveToPark;

    OuttakeSlides frontSlides, backSlides;
    PitchArm frontArm, backArm;

    EventAction slidesUpChamber, slidesDownChamber, slidesScoreChamber,
            slidesBackUpChamber, slidesBackAboveWall, slidesBackDownChamber, slidesBackScoreChamber,
            armUpChamber, armDownChamber, armDownSample, armAboveSample, armUpSample,
            armBackUpChamber, armBackDownChamber;

    ClawRotateAction clawOuttake, clawIntake, clawBackIntake, clawBackOuttake;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, -15.375, 0), telemetry);
        drive.imu.resetYaw();

        moveChamber = new P2PAction(drive, new Pose2d(-35,0,0), 5, 5);
        moveChamberAway = new P2PAction(drive, new Pose2d(-40,0, 0), 5, 5);

        moveSample1 = new P2PAction(drive, new Pose2d(-45.5, -50, 0), 5, 5);
        moveSample2 = new P2PAction(drive, new Pose2d(-45.5, -60.5, 0), 5, 5);

        moveObservation = new P2PAction(drive, new Pose2d(-54, -48, Math.toRadians(180)), 5, 5);
        moveSpecimen1 = new P2PAction(drive, new Pose2d(-59, -38, Math.toRadians(180)), 5, 5);
        moveSpecimen2 = new P2PAction(drive, new Pose2d(-63, -38, Math.toRadians(180)), 5, 5);


        moveChamber1 = new P2PAction(drive, new Pose2d(-35, -2, 0), 5, 5);
        moveChamber2 = new P2PAction(drive, new Pose2d(-45.5, -4, 0), 5, 5);

        frontSlides = new OuttakeSlides(hardwareMap, "frontSlide");
        frontArm = new PitchArm(hardwareMap, "frontPitch");
        backSlides = new OuttakeSlides(hardwareMap, "backSlide");
        backArm = new PitchArm(hardwareMap, "backPitch");

        slidesUpChamber = frontSlides.getExtensionAction(1100);
        slidesScoreChamber = frontSlides.getExtensionAction(400);
        slidesDownChamber = frontSlides.getExtensionAction(0);

        slidesBackUpChamber = backSlides.getExtensionAction(1900);
        slidesBackScoreChamber = backSlides.getExtensionAction(1000);
        slidesBackDownChamber = backSlides.getExtensionAction(0);
        slidesBackAboveWall = backSlides.getExtensionAction(350);

        //fix these
        armDownChamber = frontArm.getPitchingAction(0);
        armUpChamber = frontArm.getPitchingAction(150);

        armDownSample = frontArm.getPitchingAction(1010);
        armUpSample = frontArm.getPitchingAction(0);
        armAboveSample = frontArm.getPitchingAction(850);

        armBackDownChamber = backArm.getPitchingAction(0);
        armBackUpChamber = backArm.getPitchingAction(150);

        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.5);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);

        clawBackOuttake = new ClawRotateAction(hardwareMap, "backClaw", 0.5);
        clawBackIntake = new ClawRotateAction(hardwareMap, "backClaw", -1);

        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(drive, frontSlides, frontArm, backSlides, backArm);

        //=========TEST==========
        Follower follower = new Follower(hardwareMap);
        PositionAction posAction = new PositionAction(follower, clawBackIntake, new MultiShape(
                new Rectangle(new Point(0,30), new Point(15,0)),
                new Triangle(new Point(0, 30), new Point(0, 38), new Point(15, 30))) );
        //=======================

        Path path = new Path(new BezierLine(new Point(20, 20), new Point(40, 40)));
        path.setLinearHeadingInterpolation(0, Math.PI, 0.5);

        PedroAction betterDriveAction = new PedroAction(follower,
                new PathChain(path),
                false);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(

                new SequentialAction(

                        //score preload
                        //TODO
                        new EndAction(betterDriveAction, posAction),
                        //TODO


                        new ParallelAction(moveChamber, slidesUpChamber),
                        armUpChamber,
                        new WaitAction(500),
                        new ParallelAction(slidesScoreChamber, new TimedAction(clawOuttake, 600)),
                        new ParallelAction(moveChamberAway, slidesDownChamber, armDownChamber),

                        //get first sample and deposit in obs zone

                        moveSample1,
                        new ParallelAction(armDownSample, new TimedAction(clawIntake, 700)),
                        new ParallelAction(moveObservation, armAboveSample),
                        new TimedAction(clawBackOuttake, 700),

                        //get second sample and deposit in obs zone

                        moveSample2,
                        new ParallelAction(armDownSample, new TimedAction(clawIntake, 700)),
                        new ParallelAction(moveObservation, armAboveSample),
                        new TimedAction(clawBackOuttake, 700),

                        //Get first specimen off the wall

                        new ParallelAction(armUpSample, moveSpecimen1),
                        new ParallelAction(moveSpecimen2, new TimedAction(clawBackIntake, 1000)),

                        slidesBackAboveWall,

                        new ParallelAction(moveChamberAway, slidesBackUpChamber),
                        new ParallelAction(armBackUpChamber, moveChamber1),
                        new ParallelAction(slidesBackScoreChamber, new TimedAction(clawBackOuttake, 600)),
                        new ParallelAction(moveChamberAway, slidesBackDownChamber, armBackDownChamber),

                        //Get Second specimen off the wall

                        moveSpecimen1,
                        new ParallelAction(moveSpecimen2, new TimedAction(clawBackIntake, 1000)),

                        slidesBackAboveWall,

                        new ParallelAction(moveChamberAway, slidesBackUpChamber),
                        new ParallelAction(armBackUpChamber, moveChamber2),
                        new ParallelAction(slidesBackScoreChamber, new TimedAction(clawBackOuttake, 600)),
                        new ParallelAction(moveChamberAway, slidesBackDownChamber, armBackDownChamber),

                        moveSpecimen1




                )
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
