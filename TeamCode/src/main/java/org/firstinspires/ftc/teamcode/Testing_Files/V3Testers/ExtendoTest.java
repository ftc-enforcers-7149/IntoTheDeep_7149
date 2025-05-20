package org.firstinspires.ftc.teamcode.Testing_Files.V3Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.OuttakeExtendo;

@TeleOp(name = "Extendo Test",group="Testers")
public class ExtendoTest extends LinearOpMode {

    OuttakeExtendo extendo;

    @Override
    public void runOpMode() throws InterruptedException {

        extendo = new OuttakeExtendo(this);

        extendo.setExtension(0);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.triangle) {
                extendo.setExtension(0.15);
            } else if(gamepad1.cross) {
                extendo.setExtension(0);
            }

        }

    }
}
