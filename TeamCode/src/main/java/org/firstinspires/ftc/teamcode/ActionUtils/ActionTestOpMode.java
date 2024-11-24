package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TeleActionSequence;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.GamepadAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TeleOpLoop;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Point;


@TeleOp(name = "ActionTester")
public class ActionTestOpMode extends LinearOpMode {

    ActionManager actionManager;
    MecanumPowerDrive drive;

    WaitAction ac1;
    WaitAction ac2;

    P2PAction moveAc1, moveAc2;

    OuttakeSlides frontSlides;
    PitchArm frontArm;

    EventAction slidesUp, slidesDown, slidesScore, armUp, armDown;
    ClawRotateAction clawOuttake, clawIntake;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, 15.375, 0), telemetry);

        ac1 = new WaitAction(3000);
        ac2 = new WaitAction(6000);

        moveAc1 = new P2PAction(drive, new Pose2d(-32.75,0,0), 2, 2);
        moveAc2 = new P2PAction(drive, new Pose2d(-40,0, 0), 2, 2);

        frontSlides = new OuttakeSlides(hardwareMap, "frontSlide");
        frontArm = new PitchArm(hardwareMap, "frontPitch");

        slidesUp = frontSlides.getExtensionAction(1200);
        slidesScore = frontSlides.getExtensionAction(800);
        slidesDown = frontSlides.getExtensionAction(0);
        armDown = frontArm.getPitchingAction(0);
        armUp = frontArm.getPitchingAction(150);

        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.5);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);
        //clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.3);

        //TODO: Make an action that takes in multiple actions and runs the first once
        // until another one is triggered, interrupting the main action
        // the entire action also takes in a generic predicate (input obj is generic,
        // predicate type is generic), for an interrupt trigger to return to main
        // action execution
        // This is best applied for auto aligning with drive action running as main action
        // and using the predicate to check if the joysticks are moved, which interrupts
        // the auto align action

        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(drive, frontSlides, frontArm);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(
                new TeleOpLoop(

                        new DriveAction(drive, gamepad1, true, 1.08),

//                        new TeleInterruptActions<Gamepad>(
//                                new DriveAction(drive, gamepad1, true, 1.08),
//                                gamepad1,
//                                (gamepad ->
//                                    Math.abs(gamepad.left_stick_x) > 0.01 ||
//                                    Math.abs(gamepad.left_stick_y) > 0.01 ||
//                                    Math.abs(gamepad.right_stick_x) > 0.01
//                                ),
//                                new GamepadAction(moveAc1, gamepad1, (gamepad -> gamepad.triangle))
//                        ),
//                        new GamepadAction(new TimedAction(clawOuttake, 400), gamepad2, (gamepad -> gamepad.a && !clawIntake.isRunning)),
//                        new GamepadAction(new TimedAction(clawIntake, 1000), gamepad2, (gamepad -> gamepad.b && !clawOuttake.isRunning))

                        new TeleActionSequence(
                                new GamepadAction(slidesUp, gamepad2, (gamepad -> gamepad.a)),
                                new TeleEndAction(
                                        //the ability to run claw ends when slidesDown is first triggered
                                        //once slidesDown ends, the sequence resets to slidesUp
                                        new GamepadAction(slidesDown, gamepad2, (gamepad -> gamepad.b)),
                                        new GamepadAction(new TimedAction(new ClawRotateAction(hardwareMap, "frontClaw", 0.3), 500),
                                                gamepad1, (gamepad -> gamepad.left_trigger > 0.01))
                                )
                        )
//
//                        new GamepadAction(ac1, gamepad1, (gamepad -> gamepad.left_trigger > 0.5))

                )
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }

}
