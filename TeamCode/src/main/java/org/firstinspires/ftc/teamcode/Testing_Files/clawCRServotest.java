package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = ".CRServoTest")
public class clawCRServotest extends LinearOpMode {
    CRServo CRServo;
    Servo servoPos;
    @Override
    public void runOpMode() {
        CRServo = hardwareMap.get(CRServo.class, "servoCR");
        servoPos = hardwareMap.get(Servo.class, "servoS");

        CRServo.setPower(.5); //set to 0
        servoPos.setPosition(.5);
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
            CRServo.setPower(.5);
            servoPos.setPosition(.5);
            telemetry.addData("CurrentPower: ", CRServo.getPower());
            telemetry.addData("CurrentPosition", servoPos.getPosition());
            telemetry.update();
        }
    }
}