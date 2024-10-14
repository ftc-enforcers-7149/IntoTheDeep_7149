package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp (name = "ArmTest")
public class ArmTest extends LinearOpMode {
    Servo claw;
    Servo wristVert;
    Servo wristTwist;
    CRServo baseTwist;
    DcMotorEx baseArm, armElbow;

    @Override
    public void runOpMode() throws InterruptedException {

        baseArm = hardwareMap.get(DcMotorEx.class, "baseArm");
        //0
        armElbow = hardwareMap.get(DcMotorEx.class, "armElbow");
        //1

        claw = hardwareMap.get(Servo.class, "claw");
        //3
        wristVert = hardwareMap.get(Servo.class, "wrist");
        //1
        wristTwist = hardwareMap.get(Servo.class, "wristTwist");
        //2
        baseTwist = hardwareMap.get(CRServo.class, "baseTwist");
        //0

        claw.setPosition(0); //set to 0
        wristVert.setPosition(.5);
        wristTwist.setPosition(0);
        //baseTwist.setPosition(.5);


        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);


            baseArm.setPower(gamepad1.right_stick_y);
            //0

            armElbow.setPower(gamepad1.left_stick_y);
            //1
            claw.setPosition(gamepad1.right_trigger/2); //keep
            wristTwist.setPosition(gamepad1.left_trigger);
            wristVert.setPosition((gamepad1.left_stick_x+1)/2);
            //baseTwist.setPosition((gamepad1.right_stick_x+1)/2);
            baseTwist.setPower(gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
