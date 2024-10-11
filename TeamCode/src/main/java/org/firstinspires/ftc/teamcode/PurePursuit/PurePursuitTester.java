package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.RRTuning.Drawing;

import java.util.ArrayList;

@Config
@TeleOp(name = "PPTest")
public class PurePursuitTester extends LinearOpMode {

    ElapsedTime timer;

    public static double X = 0, Y = 0, ANG = 90;

    public static double moveSpeed = 5, turnSpeed = 5;

    public static double lookAheadDist = 5;

    MecanumPowerDrive drive;

    PurePursuitReturn ppReturn;

    int lastFoundIndex = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new MecanumPowerDrive(hardwareMap, new Pose2d(X,Y,Math.toRadians(ANG)), telemetry);
        drive.imu.resetYaw();

        ArrayList<NavPoint> path = new ArrayList<>();

        path.add(new NavPoint(X,Y, Math.toRadians(ANG)));
        path.add(new NavPoint(15, 30, Math.toRadians(90)));
        path.add(new NavPoint(35, 35, Math.toRadians(180)));
        path.add(new NavPoint(40, -10, Math.toRadians(0)));
        path.add(new NavPoint(0, -20, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while(opModeIsActive() && !isStopRequested()) {
            drive.updatePoseEstimate();

            TelemetryPacket p2pPacket = new TelemetryPacket();
            Canvas c = p2pPacket.fieldOverlay();

            ppReturn = PurePursuitController.findGoalPoint(path, drive.pose, lookAheadDist, lastFoundIndex, c);

            lastFoundIndex = ppReturn.getLastFoundIndex();

            NavPoint goal = ppReturn.getGoalPoint();

            drive.goToPose(goal, moveSpeed, turnSpeed);

            //================Displaying info on FTC Dashboard Field=============
            Drawing.drawRobot(c, drive.pose);
            c.setFill("green");
            c.fillCircle(goal.getX(), goal.getY(), 2);
            c.setStroke("green");
            c.strokeLine(goal.getX(), goal.getY(), 5 * Math.cos(goal.getHeading()) + goal.getX(), 5 * Math.sin(goal.getHeading()) + goal.getY());

            c.setStroke("red");
            for (int i = 0; i < path.size() - 1; i++) {
                NavPoint p1 = path.get(i);
                NavPoint p2 = path.get(i+1);
                c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
            }
            dashboard.sendTelemetryPacket(p2pPacket);
            //====================================================================


            telemetry.addData("xPos", drive.pose.position.x);
            telemetry.addData("yPos", drive.pose.position.y);
            telemetry.addData("angPos(RAD)", drive.pose.heading.toDouble());
            telemetry.addData("angPos(DEG)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("xTarget", X);
            telemetry.addData("yTarget", Y);
            telemetry.addData("angTarget", Math.toRadians(ANG));
            telemetry.update();
        }


    }
}
