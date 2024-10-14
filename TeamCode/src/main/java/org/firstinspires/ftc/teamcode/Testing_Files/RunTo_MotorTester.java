package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Config
@TeleOp(name = "RunTo Tester")
public class RunTo_MotorTester extends LinearOpMode {

    DcMotorEx motor;

    public static boolean reversed = false;
    public boolean manualOverride = false;

    public static int target = 0;
    public int initialPos;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initialPos = motor.getCurrentPosition();

        telemetry.addData("Initial", initialPos);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            if (reversed) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (gamepad1.triangle) {
                manualOverride = true;
            } else if (gamepad1.circle) {
                manualOverride = false;
            }

            if (!manualOverride) {
                motor.setTargetPosition(target);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
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
