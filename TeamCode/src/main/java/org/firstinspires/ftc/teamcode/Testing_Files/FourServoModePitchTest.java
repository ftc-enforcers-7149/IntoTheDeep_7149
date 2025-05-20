package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "FourServoPitch(v2.5)", group = "Testers")
public class FourServoModePitchTest extends LinearOpMode {

    ServoImplEx left1, left2, right1, right2;

    public static double servoPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left1 = hardwareMap.get(ServoImplEx.class, "left1");
        left2 = hardwareMap.get(ServoImplEx.class, "left2");
        right1 = hardwareMap.get(ServoImplEx.class, "right1");
        right2 = hardwareMap.get(ServoImplEx.class, "right2");

        left1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        left2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right2.setPwmRange(new PwmControl.PwmRange(500, 2500));


        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {


            left1.setPosition(servoPos);
            left2.setPosition(servoPos);
            right1.setPosition(servoPos);
            right2.setPosition(servoPos);



        }

    }
}
