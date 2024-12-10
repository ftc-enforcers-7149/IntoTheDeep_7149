package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TeleActionSequence extends EventAction{

    private ArrayList<EventAction> runActions;
    private ArrayList<EventAction> finishedActions;


    public TeleActionSequence(List<EventAction> actionList) {
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        runActions = new ArrayList<EventAction>((List<EventAction>) actionList);
        finishedActions = new ArrayList<>();
    }

    public TeleActionSequence(EventAction...actions) {
        this(Arrays.asList(actions));
    }


    @Override
    public boolean run(CombinedTelemetry t) {

        //when all actions are finished, this action is finished
        if (runActions.isEmpty()) {
            //reset the running actions and clear the finished actions
            runActions.addAll(finishedActions);
            finishedActions.clear();
            return false;
        }

        //run current action -> if the current action is running, this action is running
        if (runActions.get(0).run(t)) {
            return true;
        } else {
            //if the current action ended, stop it, remove it, and continue this action
            runActions.get(0).stop(false);
            finishedActions.add(runActions.remove(0));
            //if there are more actions, initialize the next one
            if (!runActions.isEmpty()) {
                runActions.get(0).init();
            }
            //if an action is finished, run the next action in the sequence
            return run(t);
        }
    }

    @Override
    public void init() {
        for (EventAction action : runActions) {
            action.init();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (!interrupted) {
            //if the action is stopped, reset the whole sequence
            runActions.addAll(finishedActions);
            finishedActions.clear();

            for (EventAction action : runActions) {
                action.stop(false);
            }
        } else {
            for (EventAction action : runActions) {
                action.stop(true);
            }
        }
    }
}
