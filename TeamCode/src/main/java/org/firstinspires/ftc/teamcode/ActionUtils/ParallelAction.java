package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction implements EventAction {

    private ArrayList<EventAction> actions;

    public ParallelAction(List<EventAction> actionList) {
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        actions = new ArrayList<EventAction>((List<EventAction>) actionList);
    }

    public ParallelAction(EventAction...actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(TelemetryPacket p) {
        //only remove an action if it is no longer running
        actions.removeIf(action -> !action.run(p));
        //if there are actions left, this action is still running
        return !actions.isEmpty();
    }

    @Override
    public void init() {
        //init all actions at once as they will all run at once
        for (EventAction act : actions){
            act.init();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        for (EventAction act : actions){
            act.stop(interrupted);
        }
    }
}
