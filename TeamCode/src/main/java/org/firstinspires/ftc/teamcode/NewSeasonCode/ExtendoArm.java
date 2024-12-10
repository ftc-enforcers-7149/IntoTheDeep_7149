package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="Extending Arm")
public class ExtendoArm extends LinearOpMode {

    CRServo servo1, servo2;

    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            servo1.setPower(gamepad1.right_stick_y);
            servo2.setPower(-1 * gamepad1.right_stick_y);

            telemetry.addData("Speed", gamepad1.right_stick_y);
            telemetry.update();
        }

    }
}
