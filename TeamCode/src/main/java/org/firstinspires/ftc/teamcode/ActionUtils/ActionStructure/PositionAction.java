package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes.Shape;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;

/**
 * Used in conjunction with movement actions to apply position activated functions during motion
 */
public class PositionAction extends EventAction {

    private EventAction action;

    private Shape triggerArea;

    private ActionLocalizer localizer;

    private boolean activated;

    public PositionAction(ActionLocalizer localizer, EventAction act, Shape area) {
        action = act;
        triggerArea = area;
        activated = false;

        this.localizer = localizer;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        if (!activated && triggerArea.withinBounds(localizer.getLocalizerPose())) {

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
