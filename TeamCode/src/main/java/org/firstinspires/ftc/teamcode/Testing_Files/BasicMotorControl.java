package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Basic Motor Control")
public class BasicMotorControl extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            double inputPower = -gamepad1.right_stick_y;

            motor.setPower(inputPower);

            telemetry.addData("Power sent to motor", inputPower);
            telemetry.addData("Power sent to motor (again)", motor.getPower());
            telemetry.addData("Position", motor.getCurrentPosition());
            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.addData("Current (amps)", motor.getCurrent(CurrentUnit.AMPS));



            telemetry.update();

        }
    }
}
