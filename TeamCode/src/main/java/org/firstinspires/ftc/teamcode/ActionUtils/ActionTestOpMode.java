package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.OuttakeSlides;
import org.firstinspires.ftc.teamcode.Hardware.PitchArm;

@Autonomous(name = "ActionTester")
public class ActionTestOpMode extends LinearOpMode {

    ActionManager actionManager;
    MecanumPowerDrive drive;

    TimerAction ac1;
    TimerAction ac2;
    TimerAction ac3;
    TimerAction ac4;

    P2PAction moveAcc;

    OuttakeSlides frontSlides;
    PitchArm frontArm;

    //EventAction slidesUp, slidesDown, armDown;

    //CubicBezier path1 = new CubicBezier(0.1, new NavPoint(-48, 0), new NavPoint(-48, 48), new NavPoint(-40, 48), new NavPoint(-48,40));

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-48, 0, Math.toRadians(90)), telemetry);

        ac1 = new TimerAction(3000, "ac1", telemetry);
        ac2 = new TimerAction(6000, "ac2", telemetry);
        ac3 = new TimerAction(2000, "ac3", telemetry);
        ac4 = new TimerAction(7000, "ac4", telemetry);

        moveAcc = new P2PAction(drive, new Pose2d(-48,24,45), 5, 5);

        //frontSlides = new OuttakeSlides(hardwareMap, "slideMotor");
        //frontArm = new PitchArm(hardwareMap, "armMotor");

        //slidesUp = frontSlides.getExtensionAction(1500);
        //slidesDown = frontSlides.getExtensionAction(0);
        //armDown = frontArm.getPitchingAction(750);

        //TODO: Make a failsafe action that takes in a trigger action and a response action,
        // response is run when the trigger is triggered, and interrupts any ongoing movement
        // actions so they can continue once the failsafe is over

        actionManager = new ActionManager(telemetry);
        actionManager.attachPeriodicActions(drive);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(
                moveAcc
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
