package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@Disabled
@TeleOp(name = "MarsRoverV2")
public class MarsRoverDrivePivot extends LinearOpMode {

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
        pitchServo.setPosition(0.5);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            // Get pitch angle
            double pitch = (imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));

            // Map angles to servo positions (assumes 0-180 degrees range)
            double pitchServoPos = (pitch + 150.0) / 300.0; // Scale to 0-1

            // Limit servo range between 0 and 1
            pitchServoPos = Math.min(Math.max(pitchServoPos, 0), 1);

            // Move servos
            pitchServo.setPosition(pitchServoPos);

            // Telemetry for debugging
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Pitch Servo", pitchServoPos);


            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double turn = gamepad1.right_stick_x;  // Left/right turn

            // Calculate power for each side
            double leftPower = drive + turn;  // Inside wheel slows down during a turn
            double rightPower = drive - turn; // Outside wheel speeds up during a turn

            // Normalize power to ensure values are within -1 to 1
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            //the front set and back set of wheels are set to the x variable of left joystick
            //the front set is going to move with the direction of the joystick
            //the back set will go the opposite direction
            leftDrive.setPower(leftPower);

            rightDrive.setPower(rightPower);

            //each side's pivot is connected to the respective joystick
            //servos should be facing directly forwards when at 0.5 position
            frontRightPivot.setPosition( (gamepad1.right_stick_x + 1) / 2 );
            frontLeftPivot.setPosition( (1 - gamepad1.right_stick_x) / 2 );


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
