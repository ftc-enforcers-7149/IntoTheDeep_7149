package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.CoreMotion.ClosedLoop.CoreSQIDF;

@Config
@TeleOp(name = "sqUIDF Tester", group = "Testers")
public class sqUIDF_MotorTester extends LinearOpMode {

    public static double kp = 0.001, ki = 0, kd = 0.001, ff = 0;

    DcMotorEx motor, motor2;

    public static boolean reversed = false;
    public static boolean reversed2 = false;
    public boolean manualOverride = false;

    public static int target = 0;
    public int initialPos;

    CoreSQIDF controller;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initialPos = motor.getCurrentPosition();

        controller = new CoreSQIDF(kp, ki, kd, ff);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && !isStopRequested()) {

            double angle = (motor.getCurrentPosition() / 990.0) * (Math.PI/2);

            if (reversed) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (reversed2) {
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            //ff *= Math.sin(angle);  //angle ff compensation

            controller.setPIDF(kp, ki, kd, ff);

            if (gamepad1.triangle) {
                manualOverride = true;
            } else if (gamepad1.circle) {
                manualOverride = false;
            }

            if (!manualOverride) {
                double power = controller.calculate(motor.getCurrentPosition() - initialPos, target);
                motor.setPower(power);
                motor2.setPower(power);
            } else {
                controller.reset();
                motor.setPower(-gamepad1.right_stick_y);
                motor2.setPower(-gamepad1.right_stick_y);
            }

            telemetry.addData("Target Position", target);
            telemetry.addData("Current Position", motor.getCurrentPosition() - initialPos);
            telemetry.addData("Manual Override", manualOverride);
            telemetry.addData("NEW VERSION", "");
            telemetry.addData("Arm Angle", angle);
            telemetry.update();

        }

    }
}
