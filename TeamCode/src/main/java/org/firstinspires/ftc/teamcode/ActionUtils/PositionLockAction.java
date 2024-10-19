package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;

public class PositionLockAction extends EventAction {

    private PIDFController xPID;
    private PIDFController yPID;
    private PIDFController angPID;

    private Pose2d target;
    private double movementSpeed, turnSpeed;

    private MecanumPowerDrive drive;


    public PositionLockAction(MecanumPowerDrive dr, Pose2d targ, double move, double turn) {
        drive = dr;
        target = targ;

        xPID = new PIDFController(MecanumPowerDrive.kpx, MecanumPowerDrive.kix, MecanumPowerDrive.kdx, 0);
        yPID = new PIDFController(MecanumPowerDrive.kpy, MecanumPowerDrive.kiy, MecanumPowerDrive.kdy, 0);
        angPID = new PIDFController(MecanumPowerDrive.kpt, MecanumPowerDrive.kit, MecanumPowerDrive.kdt, 0);

        movementSpeed = move;
        turnSpeed = turn;

    }


    @Override
    public boolean run(CombinedTelemetry t) {

        Pose2d pose = drive.pose;
        Pose2d velocity = drive.currentVelocity;
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

        //this action will always continue to lock onto a position, must be stopped by a helper action
        return true;
    }


    @Override
    public void init() {
        xPID.reset();
        yPID.reset();
        angPID.reset();
    }


    @Override
    public void stop(boolean interrupted) {
        drive.setRobotCentricPower(0, 0, 0);
    }

}
