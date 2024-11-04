package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.P2PAction;
import org.firstinspires.ftc.teamcode.ActionUtils.TimedAction;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.PitchArm;

@Disabled
@Autonomous(name = "MovementTest")
public class MovementTest extends LinearOpMode {

    ActionManager actionManager;
    MecanumPowerDrive drive;

    P2PAction moveAc1, moveAc2;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, 15.375, 0), telemetry);

        moveAc1 = new P2PAction(drive, new Pose2d(-42, 50, Math.toRadians(90)), 1, 1);
        moveAc2 = new P2PAction(drive, new Pose2d(-56, 56, Math.toRadians(135)), 1, 1);


        //TODO: Make a failsafe action that takes in a trigger action and a response action,
        // response is run when the trigger is triggered, and interrupts any ongoing movement
        // actions so they can continue once the failsafe is over

        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions(drive);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(

                new SequentialAction(
                        moveAc1,
                        moveAc2
                )

        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
