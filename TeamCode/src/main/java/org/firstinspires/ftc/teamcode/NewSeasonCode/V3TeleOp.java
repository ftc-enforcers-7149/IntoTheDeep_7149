package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ConditionalAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EndAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.GamepadActionEx;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.InstantAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TeleActionSequence;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TeleOpLoop;
import org.firstinspires.ftc.teamcode.ActionUtils.DriveAction;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.ClawDifferential;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.DoubleSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeExtendo;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.QuadServoPitch;

@TeleOp(name="V3 TeleOp")
public class V3TeleOp extends LinearOpMode {

    ActionManager actionManager;

    ClawDifferential diffyClaw;
    QuadServoPitch servoPitch;
    OuttakeExtendo outtakeExtendo;
    DoubleSlides outtakeSlides;
    MecanumPowerDrive drive;
    CRServo clawServo;

    EventAction driveAction,
            slidesUp, slidesDown,
            extendoOut, extendoIn,
            clawIntake, clawOuttake, clawStop,
            clawPitchAbove, clawPitchIntaking, clawPitchIdle, clawPitchSpec, clawRotateOuttake,
            extendoPitchAbove, extendoPitchIntaking, extendoPitchReversing,
            extendoPitchSpecGrab, extendoPitchOuttaking;

    ParallelAction intakeAbove, intakePickup, intakeReverse, returnIdle, intakeSpec;

    public enum IntakingStates{
        INTAKING, ABOVE
    }

    IntakingStates intakeStake = IntakingStates.ABOVE;
    EventAction intakingState, aboveState;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);
        diffyClaw = new ClawDifferential(this);
        servoPitch = new QuadServoPitch(this);
        outtakeExtendo = new OuttakeExtendo(this);
        outtakeSlides = new DoubleSlides(this);
        clawServo = hardwareMap.get(CRServo.class, "clawServo");

        extendoIn = new InstantAction(() -> outtakeExtendo.setExtension(0));
        extendoOut = new InstantAction(() -> outtakeExtendo.setExtension(0.2));

        clawIntake = new InstantAction(() -> clawServo.setPower(1));
        clawOuttake = new InstantAction(() -> clawServo.setPower(-0.4));
        clawStop = new InstantAction(() -> clawServo.setPower(0));

        clawPitchAbove = new InstantAction(() -> diffyClaw.setPitchAngle(90));
        clawPitchIntaking = new InstantAction(() -> diffyClaw.setPitchAngle(80));
        clawPitchIdle = new InstantAction(() -> diffyClaw.setPitchAngle(30));
        clawPitchSpec = new InstantAction(() -> diffyClaw.setPitchAngle(-40));

        clawRotateOuttake = diffyClaw.getGamepadRotationAction(gamepad2);

        driveAction = new DriveAction(drive, gamepad1, true, 1.08);

        intakeAbove = new ParallelAction(extendoPitchAbove, extendoOut, clawPitchAbove, clawStop, aboveState);
        intakePickup = new ParallelAction(extendoPitchIntaking, clawPitchIntaking, clawIntake, extendoOut, intakingState);
        intakeReverse = new ParallelAction(extendoPitchReversing, clawOuttake);
        returnIdle = new ParallelAction(extendoPitchOuttaking, extendoIn, clawPitchIdle, clawStop, aboveState);
        intakeSpec = new ParallelAction(extendoPitchSpecGrab, extendoIn, clawPitchSpec, clawIntake);

        intakingState = new InstantAction(() -> intakeStake = IntakingStates.INTAKING);
        aboveState = new InstantAction(() -> intakeStake = IntakingStates.ABOVE);

        actionManager = new ActionManager(this);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        actionManager.runActionManager(new SequentialAction(returnIdle,
                new TeleOpLoop(

                       driveAction,
                       new GamepadActionEx(gamepad1, (n) -> n.left_trigger > 0.01)
                               .whileHeld(clawOuttake),

                       //intaking section
                       new GamepadActionEx(gamepad2, (n) -> n.right_trigger > 0.01)
                               .whenPressed(intakeAbove),
                       new GamepadActionEx(gamepad2, (n) -> n.left_bumper)
                               .whenPressed(intakePickup),

                       //pulling intake up
                       new GamepadActionEx(gamepad2, (n) -> n.left_trigger > 0.01)
                               .whenPressed(new ConditionalAction(() -> {
                                   return intakeStake == IntakingStates.INTAKING;
                               }, intakeAbove, returnIdle)),

                       //specimen pickup section
                       new GamepadActionEx(gamepad2, (n) -> n.square)
                               .whenPressed(intakeSpec)





                )
        ), t -> {
            t.addData("Extendo Pitch", servoPitch.getPosition());
            t.addData("Slides Position", outtakeSlides.getCurrentPosition());
        });

    }
}
