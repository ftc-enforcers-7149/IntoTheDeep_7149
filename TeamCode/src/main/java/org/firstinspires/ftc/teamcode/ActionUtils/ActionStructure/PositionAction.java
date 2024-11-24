package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;

/**
 * Used in conjunction with movement actions to apply position activated functions during motion
 */
public class PositionAction extends EventAction {

    //TODO: Turn this into an EventAction or not?,
    // could be useful if you randomly want to execute something
    // if you happen to be in a certain position.
    // Would need to get robot pose through a
    // MecanumDrive obj or if global static odom class is used


    private EventAction action;
    private Pose2d position;
    private double tolerance;

    private MecanumPowerDrive drive;

    private boolean activated;

    public PositionAction(MecanumPowerDrive dr, EventAction act, Pose2d pos, double tol) {
        action = act;
        position = pos;
        tolerance = tol;
        activated = false;

        drive = dr;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        if (!activated && Math.hypot(position.position.x - drive.pose.position.x,
                position.position.y - drive.pose.position.y) < tolerance) {

            activated = true;
        }

        //function will only return false once the action was started and has ended

        if (activated) {
            return action.run(t);
        } else {
            return true;
        }

    }

    @Override
    public void init() {
        activated = false;
        action.init();
    }

    @Override
    public void stop(boolean interrupted) {
        activated = false;
        action.stop(interrupted);
    }


}
