package org.firstinspires.ftc.teamcode.Testing_Files.V3Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.GamepadAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.InstantAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.WaitAction;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.ClawDifferential;

@Config
@TeleOp(name="V3 Diffy Test")
public class DifferentialClawTester extends LinearOpMode {

    ClawDifferential diffyClaw;
    ActionManager manager;

    EventAction rot90, pitch90, rot45_pitch0;

    public static double rot = 0, pitch = -80;
    public static boolean posRunthrough = true;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        diffyClaw = new ClawDifferential(this);

        //TODO: This should zero the claw such that it is down all the way and rotated all of the way to the right

        diffyClaw.initPositions();
        //diffyClaw.setPitchRotAngles(-70, 0);

        rot90 = new InstantAction(() -> diffyClaw.setRotationAngle(90));
        pitch90 = new InstantAction(() -> diffyClaw.setPitchAngle(90));
        rot45_pitch0 = new InstantAction(() -> diffyClaw.setPitchRotAngles(0, 45));

        manager = new ActionManager(telemetry, hardwareMap);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

//        if (posRunthrough) {
//
//            manager.runActionManager(new SequentialAction(
//
//                    rot90,
//                    new WaitAction(2000),
//                    pitch90,
//                    new WaitAction(2000),
//                    rot45_pitch0,
//                    new WaitAction(3000)
//
//
//            ), (t) -> {
//                t.addData("Left Pos", diffyClaw.getLeftPos());
//                t.addData("Right Pos", diffyClaw.getRightPos());
//            });
//
//        }

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        while (opModeIsActive() && !isStopRequested()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            diffyClaw.setPitchRotAngles(pitch, rot);

            if (gamepad1.triangle && !previousGamepad1.triangle) {
                pitch += 5;
            } else if (gamepad1.cross && !previousGamepad1.cross) {
                pitch -= 5;
            }

            if (gamepad1.square && !previousGamepad1.square) {
                rot += 5;
            } else if (gamepad1.circle && !previousGamepad1.circle) {
                rot -= 5;
            }

            telemetry.addData("Left Pos", diffyClaw.getLeftPos());
            telemetry.addData("Right Pos", diffyClaw.getRightPos());
            telemetry.addData("Manual Mode", "");
            telemetry.addData("p: " + pitch, "r: " + rot);
            telemetry.update();
        }

    }
}
