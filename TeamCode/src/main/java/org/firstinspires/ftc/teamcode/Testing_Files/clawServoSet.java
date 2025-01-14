package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = ".5Servo")
public class clawServoSet extends LinearOpMode {
    Servo claw;

//    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "servo");
        double pos = 0.0;

        claw.setPosition(.5); //set to 0

        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
//        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);

//            test
//            if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                pos = pos +.01;
//                claw.setPosition(pos);
//            }
//            if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                pos = pos -.01;
//                claw.setPosition(pos);
//            }


            //open/close
            claw.setPosition(.5);
            telemetry.addData("CurrentPosition: ", claw.getPosition());
            telemetry.update();
        }
    }
}