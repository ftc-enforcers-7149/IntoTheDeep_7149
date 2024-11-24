package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Config
@TeleOp(name  ="Arcade-Drive", group = "Chassis")
public class ArcadeDriveTest extends LinearOpMode{

    public static double X=10, Y=25, angle=90;

    DcMotorEx frontLeft, backLeft, frontRight, backRight;

    MecanumPowerDrive drive;

    ElapsedTime pidTimer;

    boolean driverControl;
    //ServoEx drone;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        //drone = hardwareMap.get(ServoEx.class, "Drone");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90)), telemetry);

        pidTimer = new ElapsedTime();
        //drone.setPosition(0);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.triangle){
                driverControl = true;
            }

            if (gamepad1.circle) {
                driverControl = false;
            }

            drive.updatePoseEstimate();

//            if (gamepad1.touchpad) {
//                drone.setPosition(0.5);
//            }

            double y = -1 * gamepad1.left_stick_y * 1.08;
            double x = gamepad1.left_stick_x * 1.08 * 1.1;
            double rx = gamepad1.right_stick_x * 1.08;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            if (driverControl) {
                drive.goToPose(X, Y, Math.toRadians(angle), 0.3, 0.3);
            } else {
                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);
            }

            telemetry.addData("FL" ,frontLeftPower);
            telemetry.addData("PIDTime", pidTimer.milliseconds());
            telemetry.addData("P2P x", drive.pose.position.x);
            telemetry.addData("P2P y", drive.pose.position.y);
            telemetry.addData("P2P head", drive.pose.heading.toDouble());
            telemetry.addData("GoalX", X);
            telemetry.addData("GoalY", Y);
            telemetry.addData("GoalHead", angle);
            telemetry.update();

            pidTimer.reset();
        }
    }
}
