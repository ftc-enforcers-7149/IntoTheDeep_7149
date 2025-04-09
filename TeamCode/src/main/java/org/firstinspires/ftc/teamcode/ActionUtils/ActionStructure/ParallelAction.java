package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction extends EventAction {

    private ArrayList<EventAction> actions, finished;

    public ParallelAction(List<EventAction> actionList) {
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        actions = new ArrayList<EventAction>((List<EventAction>) actionList);
        finished = new ArrayList<EventAction>();
    }

    public ParallelAction(EventAction...actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(CombinedTelemetry t) {
        //end this action if there are no more actions in the list, save the actions
        if (actions.isEmpty()) {
            actions = new ArrayList<>(finished);
            finished.clear();
            return false;
        }

        //only remove an action if it is no longer running
//        actions.removeIf(action -> !action.run(t));
        for (int i = 0; i < actions.size(); i++) {
            if (!actions.get(i).run(t)) {
                finished.add(actions.remove(i));
            }
        }

        //if there are actions left, this action is still running
        return true;

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

    public ParallelAction copy() {
        return new ParallelAction(actions);
    }
}
