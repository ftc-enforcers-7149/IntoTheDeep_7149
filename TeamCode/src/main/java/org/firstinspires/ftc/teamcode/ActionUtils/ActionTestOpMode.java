package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.GamepadAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.GamepadActionEx;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.InstantAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TeleInterruptActions;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TeleOpLoop;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Trigger;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.PitchArm;


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

//        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, 15.375, 0), telemetry);
//
//        ac1 = new WaitAction(3000);
//        ac2 = new WaitAction(6000);
//
//        moveAc1 = new P2PAction(drive, new Pose2d(-32.75,0,0), 2, 2);
//        moveAc2 = new P2PAction(drive, new Pose2d(-40,0, 0), 2, 2);
//
//        frontSlides = new OuttakeSlides(hardwareMap, "frontSlide");
//        frontArm = new PitchArm(hardwareMap, "frontPitch");
//
//        slidesUp = frontSlides.getExtensionAction(1200);
//        slidesScore = frontSlides.getExtensionAction(800);
//        slidesDown = frontSlides.getExtensionAction(0);
//        armDown = frontArm.getPitchingAction(0);
//        armUp = frontArm.getPitchingAction(150);
//
//        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.5);
//        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);
//        //clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.3);
//
//        //TODO: Make gamepad actions similar to ftclib where
//        // extra methods specify if the action should run
//        // after something being pressed or held down
//        // e.g. gamepadaction.whenPressed(predicate, action).whenReleased(action2)
//        // e.g. gamepadaction.whileHeld(predicate, action).whileReleased(action2)
//        // OR
//        // connect the actions directly to a gamepad button
//        // e.g. gamepad_a.whileHeld(action).whenReleased(action2)
//        // this does prevent you from doing more complex predicates
//
//

        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(/*drive, frontSlides, frontArm*/);

        WaitAction act1 = new WaitAction(3000);
        WaitAction act2 = new WaitAction(6000);

        new GamepadActionEx(gamepad1, n -> n.triangle).whenPressed(act1);

        new GamepadActionEx(
                new Trigger<>(slidesUp, n -> !n.isRunning)
                        .and(new Trigger<>(gamepad1, n -> n.triangle)
                                .or(new Trigger<>(gamepad1, n -> n.square)))
        ).whenPressed(act1);


        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(

                new TeleOpLoop(

                        new TeleInterruptActions<Gamepad>(
                                new GamepadAction(act1, gamepad1, n -> n.triangle && !act2.isRunning),
                                gamepad1,
                                (gamepad -> Math.abs(gamepad.left_stick_x) > 0.01),
                                new GamepadAction(act2, gamepad1, n -> n.square),
                                new GamepadAction(new WaitAction(2000), gamepad1, n -> n.cross)
                        )

                        //new DriveAction(drive, gamepad1, true, 1.08),


//                        new GamepadAction(new InstantaneousAction(() -> {
//                            claw.setPower(1);
//                        }), gamepad1, gamepad -> gamepad.triangle),
//
//                        new GamepadAction(new InstantaneousAction(() -> {
//                            claw.setPower(0);
//                        }), gamepad1, gamepad -> !gamepad.triangle),

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

//                        new TeleActionSequence(
//                                new GamepadAction(slidesUp, gamepad2, (gamepad -> gamepad.a)),
//                                new TeleEndAction(
//                                        //the ability to run claw ends when slidesDown is first triggered
//                                        //once slidesDown ends, the sequence resets to slidesUp
//                                        new GamepadAction(slidesDown, gamepad2, (gamepad -> gamepad.b)),
//                                        new GamepadAction(new TimedAction(new ClawRotateAction(hardwareMap, "frontClaw", 0.3), 500),
//                                                gamepad1, (gamepad -> gamepad.left_trigger > 0.01))
//                                )
//                        )
//

                )
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }

}
