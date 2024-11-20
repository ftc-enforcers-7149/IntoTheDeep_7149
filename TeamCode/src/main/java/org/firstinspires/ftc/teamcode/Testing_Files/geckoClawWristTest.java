package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name = "geckoWrist")
public class geckoClawWristTest extends LinearOpMode {
//    CRServo claw;
    Servo wrist;

    @Override
    public void runOpMode() {
//        claw = hardwareMap.get(CRServo.class, "servo");
        wrist = hardwareMap.get(Servo.class, "wrist");

        wrist.setPosition(.5);
//        claw.setPower(0);

        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();

//        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);

            wrist.setPosition((gamepad1.left_stick_x + 1) / 2);
//            if (gamepad1.left_bumper) {
//                claw.setPower(1);
//            }
//
//            if (gamepad1.right_bumper) {
//                claw.setPower(-1);
//            }
//
//            if (gamepad1.x){
//                claw.setPower(0);
//            }

//            telemetry.addData("CurrentPosition: ", claw.getPower());
            telemetry.update();

        }
    }

}
