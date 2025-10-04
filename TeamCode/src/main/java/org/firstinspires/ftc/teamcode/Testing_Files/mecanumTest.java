package org.firstinspires.ftc.teamcode.Testing_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class mecanumTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("flMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("blMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("brMotor");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            frontLeftMotor.setPower(.1);
            backLeftMotor.setPower(.1);
            frontRightMotor.setPower(.1);
            backRightMotor.setPower(.1);
        }
    }
}