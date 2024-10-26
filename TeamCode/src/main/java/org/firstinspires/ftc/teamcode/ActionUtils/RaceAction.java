package org.firstinspires.ftc.teamcode.ActionUtils;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RaceAction extends EventAction {

    private ArrayList<EventAction> actions;

    public RaceAction(List<EventAction> actionList) {
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        actions = new ArrayList<EventAction>((List<EventAction>) actionList);
    }

    public RaceAction(EventAction...actions) {
        this(Arrays.asList(actions));
    }


    @Override
    public boolean run(CombinedTelemetry t) {
        isRunning = true;

        //run all the actions in order. The first time an action ends, end this action
        for (EventAction action : actions) {
            if (!action.run(t)) {
                stop(false);
                return false;
            }
        }

        return false;
    }

    @Override
    public void init() {
        for (EventAction action : actions) {
            action.init();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        isRunning = false;
        for (EventAction action : actions) {
            action.stop(interrupted);
        }
    }
}
