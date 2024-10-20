package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class MarsRoverDriveV2 extends LinearOpMode {

    //This opMode controls the wheels on each side at the same time (split wire)
    //and controls the pivots on each side at the same time & position (split wire)

    CRServo leftDrive, rightDrive;
    Servo leftPivot, rightPivot;


    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(CRServo.class, "leftDrive");
        rightDrive = hardwareMap.get(CRServo.class, "rightDrive");

        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");


        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //inits the servos to 0.5, which means the wheels should face forward
        //init can be used to zero the servos to this position
        leftPivot.setPosition(0.5);
        rightPivot.setPosition(0.5);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            //each side is connected to a joystick
            leftDrive.setPower(-gamepad1.left_stick_y);
            rightDrive.setPower(-gamepad1.right_stick_y);

            //each side's pivot is connected to the respective joystick
            //servos should be facing directly forwards when at 0.5 position
            leftPivot.setPosition( (gamepad1.left_stick_x + 1) / 2 );
            rightPivot.setPosition( (gamepad1.right_stick_x + 1) / 2 );


            //Use these to flip the direction of the pivots if necessary
            //leftPivot.setPosition( 1 - ((gamepad1.left_stick_x + 1) / 2) );
            //rightPivot.setPosition( 1 - ((gamepad1.right_stick_x + 1) / 2) );



            telemetry.addData("Left Joystick Up/Down", "Left Side Forward/Back");
            telemetry.addData("Right Joystick Up/Down", "Right Side Forward/Back");
            telemetry.addData("Left Joystick Left/Right", "Left Side Pivot Left/Right");
            telemetry.addData("Right Joystick Left/Right", "Right Side Left/Right");

            telemetry.addData("", "");

            telemetry.addData("left Drive", leftDrive.getPower());
            telemetry.addData("right Drive", rightDrive.getPower());
            telemetry.addData("left Pivot", leftPivot.getPosition());
            telemetry.addData("right Pivot", rightPivot.getPosition());


            telemetry.update();

        }
    }
}
