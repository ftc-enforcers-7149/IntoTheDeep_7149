package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "FourServoPitch(v2.5)", group = "Testers")
public class FourServoModePitchTest extends LinearOpMode {

    ServoImplEx left1, left2, right1, right2;

    @Override
    public void runOpMode() throws InterruptedException {

        left1 = hardwareMap.get(ServoImplEx.class, "left1");
        left2 = hardwareMap.get(ServoImplEx.class, "left2");
        right1 = hardwareMap.get(ServoImplEx.class, "right1");
        right2 = hardwareMap.get(ServoImplEx.class, "right2");

        left1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        left2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        left1.setPosition(0);
        left2.setPosition(0);
        right1.setPosition(0);
        right2.setPosition(0);


        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            if (!gamepad1.left_bumper) {

                left1.setPosition(gamepad1.left_stick_x);
                left2.setPosition(gamepad1.left_stick_x);
                right1.setPosition(gamepad1.left_stick_x);
                right2.setPosition(gamepad1.left_stick_x);

            } else {

                left1.setPosition(0.7);
                left2.setPosition(0.7);
                right1.setPosition(0.7);
                right2.setPosition(0.7);

            }

        }

    }
}
