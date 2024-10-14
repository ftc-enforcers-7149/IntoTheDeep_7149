package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.P2PAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.TimedAction;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.PitchArm;

@Autonomous(name = "BlueLeftSamples")
public class BlueLeftSamples extends LinearOpMode {

    ActionManager actionManager;
    MecanumPowerDrive drive;

    P2PAction moveAc1, moveAc2;

    OuttakeSlides frontSlides;
    PitchArm frontArm;

    EventAction slidesUp, slidesDown, armUp, armDown;
    ClawRotateAction clawOuttake, clawIntake;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, 15.375, 0), telemetry);

        moveAc1 = new P2PAction(drive, new Pose2d(-42, 50, 0), 1, 1);
        moveAc2 = new P2PAction(drive, new Pose2d(-56, 56, 135), 1, 1);

        frontSlides = new OuttakeSlides(hardwareMap, "frontSlide");
        frontArm = new PitchArm(hardwareMap, "frontPitch");

        slidesUp = frontSlides.getExtensionAction(2900);
        slidesDown = frontSlides.getExtensionAction(0);
        armDown = frontArm.getPitchingAction(1050);
        armUp = frontArm.getPitchingAction(0);

        clawOuttake = new ClawRotateAction(hardwareMap, "frontClaw", -0.5);
        clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);

        //TODO: Make a failsafe action that takes in a trigger action and a response action,
        // response is run when the trigger is triggered, and interrupts any ongoing movement
        // actions so they can continue once the failsafe is over

        actionManager = new ActionManager(telemetry);
        actionManager.attachPeriodicActions(drive, frontSlides, frontArm);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(

                new SequentialAction(
                        moveAc1,
                        new ParallelAction(armDown, new TimedAction(clawIntake, 2000)),
                        armUp,
                        moveAc2,
                        slidesUp,
                        new TimedAction(clawOuttake, 2000),
                        slidesDown
                )

        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
