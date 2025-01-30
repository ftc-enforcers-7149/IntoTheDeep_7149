package org.firstinspires.ftc.teamcode.Testing_Files.V3Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.DoubleSlides;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.QuadServoPitch;

public class V3SlidesTest extends LinearOpMode {

    public static int target = 500;
    DoubleSlides slides;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = new DoubleSlides(this);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            slides.setTarget(target);
            slides.periodic();

            telemetry.addData("Slide Position", slides.getCurrentPosition());
            telemetry.addData("Slide Target", target);
            telemetry.update();

        }

    }
}
