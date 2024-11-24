package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;
import org.firstinspires.ftc.teamcode.PathingSystems.PurePursuit.NavPoint;
import org.firstinspires.ftc.teamcode.PathingSystems.RRTuning.Drawing;
import org.firstinspires.ftc.teamcode.PathingSystems.RRTuning.MecanumDrive;

@Config
public class MecanumPowerDrive extends MecanumDrive implements PeriodicAction {

    Telemetry telemetry;
    FtcDashboard dashboard;

    public ElapsedTime updateHeading, velocityTime;

    public IMU imu;

    public Pose2d startPose, currentPose, lastPose;
    public Pose2d currentVelocity;

    //Tester chassis
//    public static double kpx = 0.1, kdx = 0.013;
//    public static double kpy = 0.12, kdy = 0.023;
//    public static double kpt = 1.1, kdt = 0.05;

    //Main chassis at 5x speed (Old)
    public static double kpx = 0.127, kdx = 0.0215, kix = 0.0055;
    public static double kpy = 0.123, kdy = 0.0219, kiy = 0.004;
    public static double kpt = 1.56, kdt = 0.089, kit = 0.015;

    //Main chassis at 5x speed
//    public static double kpx = 0.071, kdx = 0.012, kix = 0.018;
//    public static double kpy = 0.056, kdy = 0.012, kiy = 0.008;
//    public static double kpt = 0.54, kdt = 0.05, kit = 0.045;

    //Main chassis at x1 speed
//    public static double kpx = 0.11, kdx = 0.011, kix = 0;
//    public static double kpy = 0.13, kdy = 0.02, kiy = 0;
//    public static double kpt = 1.22, kdt = 0.064, kit = 0;

    public PIDFController xPID = new PIDFController(kpx, 0, kdx, 0);
    public PIDFController yPID = new PIDFController(kpy, 0, kdy, 0);
    public PIDFController angPID = new PIDFController(kpt, 0, kdt, 0);

    public MecanumPowerDrive(HardwareMap hMap, Pose2d startPos, Telemetry tel){
        super(hMap, startPos);
        startPose = startPos;
        telemetry = tel;
        initializeIMU(hMap);
        dashboard = FtcDashboard.getInstance();

        updateHeading = new ElapsedTime();
        velocityTime = new ElapsedTime();

        currentPose = startPose;
        lastPose = startPose;
    }

    public void initializeIMU(HardwareMap hmap) {
        imu = hmap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(imuParams);
    }



    public void setRobotCentricPower(double x, double y, double rx){

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        //telemetry.addData("DENOMINATOR", denominator);
    }

    public void setFieldCentricPower(double x, double y, double rx){

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    /**
     * Wraps an angle within the range -180 to +180 degrees
     * @param angle
     * @return Wrapped angle
     */
    public static double AngleWrap(double angle) {
        return angle % 180;
    }


    public void goToPose(NavPoint pt, double movementSpeed, double turnSpeed) {
        goToPose(pt.getX(), pt.getY(), pt.getHeading(), movementSpeed, turnSpeed);
    }

    /**
     * Returns the x, y, and turn movement powers necessary to move the robot to a target position (x,y,theta)
     * @param x The x position of the target point
     * @param y The y position of the target point
     * @param preferredAngle The angle towards which the robot should rotate while approaching the target position
     * @param movementSpeed Range between 0.0 --> 1.0 at which the x and y movement powers are multiplied by
     * @param turnSpeed Range between 0.0 --> 1.0 at which the turn power is multiplied by
     * @return A 3 dimensional vector representing the x movement power, y movement power, and turn movement power (following simple mechanum drive kinematics)
     */

    public void goToPose(double x, double y, double preferredAngle, double movementSpeed, double turnSpeed) {

//        //Displaying info on FTC Dashboard Field
//        TelemetryPacket p2pPacket = new TelemetryPacket();
//        Canvas c = p2pPacket.fieldOverlay();
//        Drawing.drawRobot(c, pose);
//        c.fillCircle(x, y, 3);
//        c.strokeLine(x, y, 5 * Math.cos(preferredAngle) + x, 5 * Math.sin(preferredAngle) + y);
//        dashboard.sendTelemetryPacket(p2pPacket);

        xPID.setPIDF(kpx, kix, kdx, 0);
        yPID.setPIDF(kpy, kiy, kdy, 0);
        angPID.setPIDF(kpt, kit, kdt, 0);

        double heading = pose.heading.toDouble();
        telemetry.addData("p2p HEADING", heading);

        if (preferredAngle - heading > Math.PI) preferredAngle -= 2 * Math.PI;
        if (preferredAngle - heading < -Math.PI) preferredAngle += 2 * Math.PI;

        double xPower = xPID.calculate(pose.position.x, x);
        double yPower = yPID.calculate(pose.position.y, y);
        double hPower = -angPID.calculate(heading, preferredAngle);

        telemetry.addData("PID POWERS", xPower + "\n" + yPower + "\n" + hPower);

        double xRot = xPower * Math.cos(Math.toRadians(90) - heading) - yPower * Math.sin(Math.toRadians(90) - heading);
        double yRot = xPower * Math.sin(Math.toRadians(90) - heading) + yPower * Math.cos(Math.toRadians(90) - heading);

        setRobotCentricPower(xRot * movementSpeed, yRot * movementSpeed, hPower * turnSpeed);
    }

    @Override
    public void periodic() {
        this.updatePoseEstimate();

        lastPose = currentPose;
        currentPose = pose;

        double velocityLoopTime = velocityTime.seconds();
        currentVelocity = new Pose2d( (lastPose.position.x - currentPose.position.x) / velocityLoopTime, (lastPose.position.y - currentPose.position.y) / velocityLoopTime, (lastPose.heading.toDouble() - currentPose.heading.toDouble()) / velocityLoopTime);

        if (updateHeading.milliseconds() > 250) {
            //resets heading using the imu every 1/4 second
            this.pose = new Pose2d(this.pose.position.x, this.pose.position.y, AngleWrap(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + startPose.heading.toDouble()));
            updateHeading.reset();
        }

        TelemetryPacket t = new TelemetryPacket();
        Canvas c = t.fieldOverlay();
        Drawing.drawRobot(c, pose);
        FtcDashboard.getInstance().sendTelemetryPacket(t);

        telemetry.addData("Drive Position", this.pose.position.x + "  " + this.pose.position.y + "  " + Math.toDegrees(this.pose.heading.toDouble()));
        telemetry.addData("Drive Velocity", currentVelocity.position.x + "  " + currentVelocity.position.y + "  " + Math.toDegrees(currentVelocity.heading.toDouble()));
    }

}
