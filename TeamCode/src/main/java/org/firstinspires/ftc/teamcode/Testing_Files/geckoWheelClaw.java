package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name = "VikramClaw")
public class geckoWheelClaw extends LinearOpMode {
    CRServo claw;

    @Override
    public void runOpMode() {
        claw = hardwareMap.get(CRServo.class, "servo");

        claw.setPower(0);

        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
//        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);


            if (gamepad1.left_bumper) {
                claw.setPower(1);
            }

            if (gamepad1.right_bumper) {
                claw.setPower(-1);
            }

            if (gamepad1.x){
                claw.setPower(0);
            }

            telemetry.addData("CurrentPosition: ", claw.getPower());
            telemetry.update();

        }
    }

}
