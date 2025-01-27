package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.InstantAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.WaitAction;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.ClawDifferential;

@TeleOp(name="V3 Diffy Test", group="Testers")
public class DifferentialClawTester extends LinearOpMode {

    ClawDifferential diffyClaw;
    ActionManager manager;

    EventAction rot90, pitch90, rot45_pitch0;

    @Override
    public void runOpMode() throws InterruptedException {

        diffyClaw = new ClawDifferential(this);
        diffyClaw.initPositions();

        rot90 = new InstantAction(() -> diffyClaw.setRotationAngle(90));
        pitch90 = new InstantAction(() -> diffyClaw.setPitchAngle(90));
        rot45_pitch0 = new InstantAction(() -> diffyClaw.setPitchRotAngles(0, 45));

        manager = new ActionManager(telemetry, hardwareMap);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        manager.runActionManager(new SequentialAction(

                rot90,
                new WaitAction(2000),
                pitch90,
                new WaitAction(2000),
                rot45_pitch0,
                new WaitAction(3000)


        ), (t) -> {
            t.addData("Left Pos", diffyClaw.getLeftPos());
            t.addData("Right Pos", diffyClaw.getRightPos());
        });

    }
}
