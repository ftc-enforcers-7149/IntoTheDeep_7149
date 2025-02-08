package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.LLSampleVision;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;
//import org.firstinspires.ftc.teamcode.Hardware.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.util.SingleRunAction;

import java.util.ArrayList;

@TeleOp(name = "backArmSet")
public class backArmSet extends LinearOpMode {
    Servo claw;
    DcMotorEx pitchMotorBack;
    PIDFController slidesBack;

    //    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "servo");
        double pos = 0.0;

        claw.setPosition(0); //set to 0

        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
//        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);

            //sample pickup (slightly above 90 degrees)
            if (gamepad1.dpad_up ) {
                claw.setPosition(.36);
            } //90

            if (gamepad1.dpad_down) {
                claw.setPosition(0);
            }


            if (gamepad1.dpad_left)




            //scoring for chamber
            if (gamepad1.dpad_right) {
                claw.setPosition(.125); //good
            }



            //open/close
//            claw.setPosition(0);
            telemetry.addData("CurrentPosition: ", claw.getPosition());
            telemetry.update();
        }
    }
}