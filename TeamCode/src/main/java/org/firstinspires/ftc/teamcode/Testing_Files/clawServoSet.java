package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp(name = "Servo Tester")
public class clawServoSet extends LinearOpMode {

    ServoImplEx claw;

    @Override
    public void runOpMode() {
        claw = hardwareMap.get(ServoImplEx.class, "servo");
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));

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


            if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
                pos = pos + .01;
                claw.setPosition(pos);
            }
            if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
                pos = pos -.01;
                claw.setPosition(pos);
            }


            //open/close
            telemetry.addData("CurrentPosition: ", claw.getPosition());
            telemetry.update();
        }
    }
}