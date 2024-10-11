package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp (name = "ScoreArmTestPID")
public class scoringArmTestPID extends LinearOpMode{
    DcMotorEx elevator;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;

    public static double ff = 0;

    public static int target = 300;
    public static int targetPosition = 0;
    public int initalPos;
    ElapsedTime timer = new ElapsedTime();
    PIDFController elevatorController;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator = hardwareMap.get(DcMotorEx.class, "baseArm");

        elevatorController = new PIDFController(Kp, Ki, Kd, ff);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        initalPos = elevator.getCurrentPosition();
        waitForStart();

        while (opModeIsActive()) {

//            elevatorController.setPIDF(Kp, Ki, Kd, ff);
//
//            double power = elevatorController.calculate(elevator.getCurrentPosition() - initalPos,target);
//            elevator.setPower(power/5);
            elevator.setTargetPosition(targetPosition);

            telemetry.addData("CurrentPosition", elevator.getCurrentPosition() - initalPos);
            telemetry.addData("TargetPosition", target);
            telemetry.update();

        }
    }
}
