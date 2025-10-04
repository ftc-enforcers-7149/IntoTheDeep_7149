package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Subsystems.SwerveModule;

@TeleOp(name = "Swerve Pod Test", group="Testers")
public class SwervePodTest extends LinearOpMode {

    SwerveModule pod;

    public static double motorPow = 0;
    public static double servoPow = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        pod = new SwerveModule(hardwareMap, "motor", "servo");

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            double servoPos = gamepad1.left_stick_x * 180;
            telemetry.addData("Servo Position", servoPos);

            double motorPow = gamepad1.right_stick_x;
            telemetry.addData("Motor Power", motorPow);

            pod.setModuleState(motorPow, servoPos);

            telemetry.addData("Actual Servo Position", pod.getServoPosition());

            telemetry.update();

        }

    }
}