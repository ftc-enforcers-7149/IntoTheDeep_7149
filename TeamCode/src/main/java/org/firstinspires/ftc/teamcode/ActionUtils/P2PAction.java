package org.firstinspires.ftc.teamcode.ActionUtils;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;

//TODO: Have the p2p action take in position activated functions
// that run in parallel with the main motion action,
// Checks if each position function is within its tolerance and activates it

//TODO: Might need to make a position activated wrapper class for an action
public class P2PAction implements EventAction {

    private PIDFController xPID;
    private PIDFController yPID;
    private PIDFController angPID;

    private Pose2d target;
    private double movementSpeed, turnSpeed;

    private MecanumPowerDrive drive;


    public P2PAction(MecanumPowerDrive dr, Pose2d targ, double move, double turn) {
        drive = dr;
        target = targ;

        xPID = new PIDFController(MecanumPowerDrive.kpx, 0, MecanumPowerDrive.kdx, 0);
        yPID = new PIDFController(MecanumPowerDrive.kpy, 0, MecanumPowerDrive.kdy, 0);
        angPID = new PIDFController(MecanumPowerDrive.kpt, 0, MecanumPowerDrive.kdt, 0);

        movementSpeed = move;
        turnSpeed = turn;

    }


    @Override
    public boolean run(TelemetryPacket p) {

        Pose2d pose = drive.pose;
        double preferredAngle = target.heading.toDouble();

        double heading = pose.heading.toDouble();

        if (preferredAngle - heading > Math.PI) preferredAngle -= 2 * Math.PI;
        if (preferredAngle - heading < -Math.PI) preferredAngle += 2 * Math.PI;

        double xPower = xPID.calculate(pose.position.x, target.position.x);
        double yPower = yPID.calculate(pose.position.y, target.position.y);
        double hPower = -angPID.calculate(heading, preferredAngle);

        double xRot = xPower * Math.cos(Math.toRadians(90) - heading) - yPower * Math.sin(Math.toRadians(90) - heading);
        double yRot = xPower * Math.sin(Math.toRadians(90) - heading) + yPower * Math.cos(Math.toRadians(90) - heading);

        drive.setRobotCentricPower(xRot * movementSpeed, yRot * movementSpeed, hPower * turnSpeed);

        //if off target or heading is incorrect, continue the action
        return Math.hypot(pose.position.x - target.position.x, pose.position.y - target.position.y) > 0.1 || Math.abs(preferredAngle - heading) > 0.1;
    }


    @Override
    public void init() {
        xPID.reset();
        yPID.reset();
        angPID.reset();
    }

    @Override
    public void stop(boolean interrupted) {
        drive.setRobotCentricPower(0,0,0);

    }

}
