package org.firstinspires.ftc.teamcode.Testing_Files.V3Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.QuadServoPitch;

@Config
@TeleOp(name="V3 Pitch Test", group="Testers")
public class V3PitchTest extends LinearOpMode {

    public static int target = 500;
    QuadServoPitch pitchSubsystem;
    public boolean manual = false;

    @Override
    public void runOpMode() throws InterruptedException {

        pitchSubsystem = new QuadServoPitch(this);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while(opModeIsActive() && !isStopRequested()) {

            if (gamepad1.triangle) {
                manual = true;
            } else if (gamepad1.circle) {
                manual = false;
            }

            if (manual) {
                pitchSubsystem.setPower(gamepad1.left_stick_x);
            } else {
                pitchSubsystem.setTarget(target);
                pitchSubsystem.periodic();
            }


            telemetry.addData("Pitch Position", pitchSubsystem.getPosition());
            telemetry.addData("Pitch Target", target);
            telemetry.update();

        }

    }
}
