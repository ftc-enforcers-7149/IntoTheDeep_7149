package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "MarsRoverTuning")
public class MarsRoverTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo ldb = hardwareMap.get(CRServo.class, "ldb");
        CRServo ldm = hardwareMap.get(CRServo.class, "ldm");
        CRServo ldf = hardwareMap.get(CRServo.class, "ldf");
        CRServo rdb = hardwareMap.get(CRServo.class, "rdb");
        CRServo rdm = hardwareMap.get(CRServo.class, "rdm");
        CRServo rdf = hardwareMap.get(CRServo.class, "rdf");

        CRServo pitchm = hardwareMap.get(CRServo.class, "pitchm");

        ldb.setDirection(CRServo.Direction.REVERSE);
        ldm.setDirection(CRServo.Direction.REVERSE);
        ldf.setDirection(CRServo.Direction.REVERSE);

        Servo frontRightPivot = hardwareMap.get(Servo.class, "frSteer");
        Servo frontLeftPivot = hardwareMap.get(Servo.class, "flSteer");
        Servo backRightPivot = hardwareMap.get(Servo.class, "brSteer");
        Servo backLeftPivot = hardwareMap.get(Servo.class, "blSteer");

        double frpos = .5;
        double flpos = .5;
        double brpos = .5;
        double blpos = .5;

        frontRightPivot.setPosition(frpos);
        frontLeftPivot.setPosition(flpos);
        backRightPivot.setPosition(brpos);
        backLeftPivot.setPosition(blpos);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (!gamepad1.cross) {
                if (gamepad1.dpad_up) {
                    frpos += .01;
                    sleep(500);
                }
                if (gamepad1.dpad_left) {
                    flpos += .01;
                    sleep(500);
                }
                if (gamepad1.dpad_right) {
                    brpos += .01;
                    sleep(500);
                }
                if (gamepad1.dpad_down) {
                    blpos += .01;
                    sleep(500);
                }
            } else if (gamepad1.cross) {
                if (gamepad1.dpad_up) {
                    frpos -= .01;
                    sleep(500);
                }
                if (gamepad1.dpad_left) {
                    flpos -= .01;
                    sleep(500);
                }
                if (gamepad1.dpad_right) {
                    brpos -= .01;
                    sleep(500);
                }
                if (gamepad1.dpad_down) {
                    blpos -= .01;
                    sleep(500);
                }
            }

            frpos = Math.max(0.0, Math.min(1.0, frpos));
            flpos = Math.max(0.0, Math.min(1.0, flpos));
            brpos = Math.max(0.0, Math.min(1.0, brpos));
            blpos = Math.max(0.0, Math.min(1.0, blpos));

            frontRightPivot.setPosition(frpos);
            frontLeftPivot.setPosition(flpos);
            backRightPivot.setPosition(brpos);
            backLeftPivot.setPosition(blpos);

            telemetry.addData("Front Right Pos", frpos);
            telemetry.addData("Front Left Pos", flpos);
            telemetry.addData("Back Right Pos", brpos);
            telemetry.addData("Back Left Pos", blpos);
            telemetry.update();
        }
    }
}
