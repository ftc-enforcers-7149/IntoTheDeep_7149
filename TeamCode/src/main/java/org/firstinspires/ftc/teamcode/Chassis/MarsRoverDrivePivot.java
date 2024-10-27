package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MarsRoverV1.5")
public class MarsRoverDrivePivot extends LinearOpMode {

    //This opMode controls the wheels on each side at the same time (split wire)
    //and controls the pivots on each side at the same time & position (split wire)

    CRServo leftDrive, rightDrive;
    Servo frontRightPivot, frontLeftPivot;


    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(CRServo.class, "leftDrive");
        rightDrive = hardwareMap.get(CRServo.class, "rightDrive");

        frontRightPivot = hardwareMap.get(Servo.class, "rightLeftPivot");
        frontLeftPivot = hardwareMap.get(Servo.class, "leftRightPivot");


        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //inits the servos to 0.5, which means the wheels should face forward
        //init can be used to zero the servos to this position
        frontRightPivot.setPosition(0.5);
        frontLeftPivot.setPosition(0.5);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            //the front set and back set of wheels are set to the x variable of left joystick
            //the front set is going to move with the direction of the joystick
            //the back set will go the opposite direction
            leftDrive.setPower(-gamepad1.right_stick_y);

            rightDrive.setPower(-gamepad1.right_stick_y);

            //each side's pivot is connected to the respective joystick
            //servos should be facing directly forwards when at 0.5 position
            frontRightPivot.setPosition( (gamepad1.left_stick_x + 1) / 2 );
            frontLeftPivot.setPosition( (1 - gamepad1.left_stick_x) / 2 );


            //Use these to flip the direction of the pivots if necessary
            //leftPivot.setPosition( 1 - ((gamepad1.left_stick_x + 1) / 2) );
            //rightPivot.setPosition( 1 - ((gamepad1.right_stick_x + 1) / 2) );



            telemetry.addData("Right Joystick Up/Down", "Forward/Backward");
            telemetry.addData("Left Joystick Left/Right", "Robot Steering Left/Right");

            telemetry.addData("", "");

            telemetry.addData("left Drive", leftDrive.getPower());
            telemetry.addData("right Drive", rightDrive.getPower());
            telemetry.addData("left front Pivot", frontLeftPivot.getPosition());
            telemetry.addData("right front Pivot", frontLeftPivot.getPosition());


            telemetry.update();

        }
    }
}
