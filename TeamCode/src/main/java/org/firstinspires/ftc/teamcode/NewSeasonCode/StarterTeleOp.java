package org.firstinspires.ftc.teamcode.NewSeasonCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;

@Config
@TeleOp(name = "TesterTeleOp")
public class StarterTeleOp extends LinearOpMode {

    MecanumPowerDrive drive;

    Servo claw;

    DcMotorEx slideMotor, armMotor;

    protected enum GoStage {
        Zero,
        One,
        Two;
    }

    protected GoStage Stage = GoStage.Zero;

    public static double kP = 0, kI = 0, kD = 0, ff = 0;

    public static int targetPosition = 400;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,0), telemetry);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {


            drive.setRobotCentricPower(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            slideMotor.setPower(gamepad1.right_stick_y);

            if (gamepad1.left_trigger > 0.05) {
                armMotor.setPower(gamepad1.left_trigger/2);
            }

            if (gamepad1.right_trigger > 0.05) {
                armMotor.setPower(-gamepad1.right_trigger/2);
            }

            if (gamepad1.dpad_up) {
                claw.setPosition(.27);
            }

            if (gamepad1.dpad_down) {
                claw.setPosition(0);
            }



        }

    }
}
