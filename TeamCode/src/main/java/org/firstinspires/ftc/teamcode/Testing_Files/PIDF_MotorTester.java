package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "PIDF Tester")
public class PIDF_MotorTester extends LinearOpMode {

    public static double kp = 0.001, ki = 0, kd = 0.001, ff = 0;

    DcMotorEx motor;

    public static boolean reversed = false;
    public boolean manualOverride = false;

    public static int target = 0;
    public int initialPos;

    PIDFController controller;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initialPos = motor.getCurrentPosition();

        controller = new PIDFController(kp, ki, kd, ff);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {

            if (reversed) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            controller.setPIDF(kp, ki, kd, ff);

            if (gamepad1.triangle) {
                manualOverride = true;
            } else if (gamepad1.circle) {
                manualOverride = false;
            }

            if (!manualOverride) {
                double power = controller.calculate(motor.getCurrentPosition() - initialPos, target);
                motor.setPower(power);
            } else {
                controller.reset();
                motor.setPower(-gamepad1.right_stick_y);
            }

            telemetry.addData("Target Position", target);
            telemetry.addData("Current Position", motor.getCurrentPosition() - initialPos);
            telemetry.addData("Manual Override", manualOverride);
            telemetry.addData("NEW VERSION", "");
            telemetry.update();

        }

    }
}
