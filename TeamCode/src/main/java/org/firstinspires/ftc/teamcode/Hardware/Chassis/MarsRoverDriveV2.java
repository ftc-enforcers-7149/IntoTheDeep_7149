package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MarsRoverV2.0")

public class MarsRoverDriveV2 extends LinearOpMode {

    //This opMode controls the wheels on each side at the same time (split wire)
    //and controls the pivots on each side at the same time & position (split wire)

    CRServo leftDrive, rightDrive;
    Servo frontRightPivot, frontLeftPivot, pitchServo;
    IMU imu;


    //    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(CRServo.class, "leftDrive");
        rightDrive = hardwareMap.get(CRServo.class, "rightDrive");

        //for current set up, steering should be 2 front wheels in same direction, back 2 wheels in same direction
        //front steer is front right 
        frontRightPivot = hardwareMap.get(Servo.class, "frontSteer");
        frontLeftPivot = hardwareMap.get(Servo.class, "backSteer");

        //for the pitch balancing
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        imu = hardwareMap.get(IMU.class, "imu");


        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //inits the servos to 0.5, which means the wheels should face forward
        //init can be used to zero the servos to this position
        frontRightPivot.setPosition(0.5);
        frontLeftPivot.setPosition(0.5);

        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(imuParams);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            // Get pitch angle
            double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);

            // Map angles to servo positions (assumes 0-180 degrees range)
            double pitchServoPos = (pitch + 90) / 180.0; // Scale to 0-1

            // Limit servo range between 0 and 1
            pitchServoPos = Math.min(Math.max(pitchServoPos, 0), 1);

            // Move servos
            pitchServo.setPosition(pitchServoPos);

            // Telemetry for debugging
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Pitch Servo", pitchServoPos);




            //the front set and back set of wheels are set to the x variable of left joystick
            //the front set is going to move with the direction of the joystick
            //the back set will go the opposite direction
            leftDrive.setPower(-gamepad1.right_stick_y);

            rightDrive.setPower(-gamepad1.right_stick_y);

            //each side's pivot is connected to the respective joystick
            //servos should be facing directly forwards when at 0.5 position
            //left
            if (gamepad1.left_stick_x > 0.05){
               frontRightPivot.setPosition(-1);
               frontLeftPivot.setPosition(1);
               rightDrive.setPower(-gamepad1.left_stick_x);
               leftDrive.setPower(gamepad1.left_stick_x);
            } else if (gamepad1.left_stick_x < -0.05){
                //right
                frontRightPivot.setPosition(-1);
                frontLeftPivot.setPosition(1);
                rightDrive.setPower(-gamepad1.left_stick_x);
                leftDrive.setPower(gamepad1.left_stick_x);
            } else{
                frontRightPivot.setPosition(.5);
                frontLeftPivot.setPosition(.5);

            }
            telemetry.addData("left joystick x", gamepad1.left_stick_x);

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

//    //This opMode controls the wheels on each side at the same time (split wire)
//    //and controls the pivots on each side at the same time & position (split wire)
//
//    CRServo leftDrive, rightDrive;
//    Servo leftPivot, rightPivot;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        leftDrive = hardwareMap.get(CRServo.class, "leftDrive");
//        rightDrive = hardwareMap.get(CRServo.class, "rightDrive");
//
//        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
//        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
//
//
//        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //inits the servos to 0.5, which means the wheels should face forward
//        //init can be used to zero the servos to this position
//        leftPivot.setPosition(0.5);
//        rightPivot.setPosition(0.5);
//
//        waitForStart();
//
//        if (isStopRequested()) {
//            return;
//        }
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//            //each side is connected to a joystick
//            leftDrive.setPower(-gamepad1.left_stick_y);
//            rightDrive.setPower(-gamepad1.right_stick_y);
//
//            //each side's pivot is connected to the respective joystick
//            //servos should be facing directly forwards when at 0.5 position
//            leftPivot.setPosition( (gamepad1.left_stick_x + 1) / 2 );
//            rightPivot.setPosition( (gamepad1.right_stick_x + 1) / 2 );
//
//
//            //Use these to flip the direction of the pivots if necessary
//            //leftPivot.setPosition( 1 - ((gamepad1.left_stick_x + 1) / 2) );
//            //rightPivot.setPosition( 1 - ((gamepad1.right_stick_x + 1) / 2) );
//
//
//
//            telemetry.addData("Left Joystick Up/Down", "Left Side Forward/Back");
//            telemetry.addData("Right Joystick Up/Down", "Right Side Forward/Back");
//            telemetry.addData("Left Joystick Left/Right", "Left Side Pivot Left/Right");
//            telemetry.addData("Right Joystick Left/Right", "Right Side Left/Right");
//
//            telemetry.addData("", "");
//
//            telemetry.addData("left Drive", leftDrive.getPower());
//            telemetry.addData("right Drive", rightDrive.getPower());
//            telemetry.addData("left Pivot", leftPivot.getPosition());
//            telemetry.addData("right Pivot", rightPivot.getPosition());
//
//
//            telemetry.update();
//
//        }
//    }
//}
