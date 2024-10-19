package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.PurePursuit.HermiteCurve;
import org.firstinspires.ftc.teamcode.PurePursuit.NavPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitReturn;
import org.firstinspires.ftc.teamcode.RRTuning.Drawing;

public class PurePursuitAction extends EventAction {

    private HermiteCurve curve;
    private MecanumPowerDrive drive;

    private double moveSpeed, turnSpeed;
    private int lastFoundIndex;
    private double lookAhead;


    //TODO: decide whether speed will be constant for a path or differ depending on the navPoint
    public PurePursuitAction(MecanumPowerDrive dr, HermiteCurve crv, double lookAheadRadius, double move, double turn) {
        drive = dr;
        curve = crv;
        moveSpeed = move;
        turnSpeed = turn;

        lastFoundIndex = 0;
        lookAhead = lookAheadRadius;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        Canvas c = t.getPacket().fieldOverlay();

        PurePursuitReturn ptAndLastFound =
                PurePursuitController.findGoalPoint(curve.getCurvePoints(), drive.pose, lookAhead, lastFoundIndex, c);

        lastFoundIndex = ptAndLastFound.getLastFoundIndex();

        NavPoint goalPt = ptAndLastFound.getGoalPoint();

        P2PAction goToGoal = new P2PAction(drive, goalPt.toPose(), moveSpeed, turnSpeed);

        //==========DRAWING FOR FIELD DISPLAY=============

        Drawing.drawRobot(c, drive.pose);
            c.setFill("green");
            c.fillCircle(goalPt.getX(), goalPt.getY(), 2);
            c.setStroke("green");
            c.strokeLine(goalPt.getX(), goalPt.getY(), 5 * Math.cos(goalPt.getHeading()) + goalPt.getX(), 5 * Math.sin(goalPt.getHeading()) + goalPt.getY());

        c.setStroke("red");
        for (int i = 0; i < curve.getSize() - 1; i++) {
            NavPoint p1 = curve.getCurvePoints().get(i);
            NavPoint p2 = curve.getCurvePoints().get(i+1);
            c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
        }

        //=============END OF DRAWING=====================

        //Will only return true when the robot comes to a complete stop from a p2p action
        //and it has reached the final point in the path
        return goToGoal.run(t) && lastFoundIndex >= curve.getSize() - 1;
    }

    @Override
    public void init() {

    }

    @Override
    public void stop(boolean interrupted) {
        drive.setRobotCentricPower(0,0,0);
    }
}
