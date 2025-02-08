package org.firstinspires.ftc.teamcode.Testing_Files;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Hardware.Subsystems.VisionSubsystem;


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