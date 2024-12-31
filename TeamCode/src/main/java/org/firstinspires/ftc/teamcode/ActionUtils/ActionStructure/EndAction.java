package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class EndAction extends EventAction {

    private ArrayList<EventAction> secondaryActions;
    private EventAction firstAction;


    public EndAction(EventAction action1, List<EventAction> actionList) {
        firstAction = action1;
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        secondaryActions = new ArrayList<EventAction>((List<EventAction>) actionList);
    }

    public EndAction(EventAction action1, EventAction...actions) {
        this(action1, Arrays.asList(actions));
    }


    @Override
    public boolean run(CombinedTelemetry t) {
        isRunning = true;

        //when the first action ends, end all of the other actions
        boolean finished = firstAction.run(t);

        if (finished) {
            stop(false);
            return false;
        } else {
            //run the other actions in parallel
            for (EventAction action : secondaryActions) {
                action.run(t);
            }
            return true;
        }

    }

    @Override
    public void init() {
        firstAction.init();
        for (EventAction action : secondaryActions) {
            action.init();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        isRunning = false;

        firstAction.stop(interrupted);
        for (EventAction action : secondaryActions) {
            action.stop(interrupted);
        }
    }
}
