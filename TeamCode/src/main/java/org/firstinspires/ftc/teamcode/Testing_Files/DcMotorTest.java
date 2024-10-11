package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DcMotorTester", group = "TesterFiles")
public class DcMotorTest extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            motor.setPower(gamepad1.right_stick_y);

            telemetry.addData("Move the right stick", "");
            telemetry.addData("Up is Negative Power", "");
            telemetry.addData("Down is Positive Power", "");
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Motor Position", motor.getCurrentPosition());

            telemetry.update();

        }
    }
}
