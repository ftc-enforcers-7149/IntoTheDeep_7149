package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Subsystems.QuadServoPitch;

@TeleOp(name="V3 Pitch Test", group="Testers")
public class V3PitchTest extends LinearOpMode {

    QuadServoPitch pitchSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {

        pitchSubsystem = new QuadServoPitch(this);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while(opModeIsActive() && !isStopRequested()) {

            pitchSubsystem.setPower(gamepad1.left_stick_x);

            telemetry.addData("Pitch Position", pitchSubsystem.getPosition());
            telemetry.update();

        }

    }
}
