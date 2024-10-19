package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.GamepadAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TeleOpLoop;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.PitchArm;



@Autonomous(name = "ActionTester")
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

    //CubicBezier path1 = new CubicBezier(0.1, new NavPoint(-48, 0), new NavPoint(-48, 48), new NavPoint(-40, 48), new NavPoint(-48,40));

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

        //TODO: Make a failsafe action that takes in a normal action and a response action,
        // response is run when the failsafe is triggered, and interrupts the normal action

        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(drive, frontSlides, frontArm);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(
                new TeleOpLoop(

                        new GamepadAction(new DriveAction(drive, gamepad1, true, 1.08), gamepad1, (gamepad -> true)),
                        new GamepadAction(slidesUp, gamepad1, (gamepad -> gamepad.a && !slidesDown.isRunning)),
                        new GamepadAction(slidesDown, gamepad1, (gamepad -> gamepad.b && !slidesUp.isRunning)),

                        new GamepadAction(ac1, gamepad1, (gamepad -> gamepad.right_trigger > 0.5))


                )
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }

}
