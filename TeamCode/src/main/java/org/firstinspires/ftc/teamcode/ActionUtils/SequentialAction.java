package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequentialAction implements EventAction {

    ArrayList<EventAction> actions;

    public SequentialAction(List<EventAction> actionList) {
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        actions = new ArrayList<EventAction>((List<EventAction>) actionList);
    }

    public SequentialAction(EventAction...actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(TelemetryPacket p) {
        //when all actions are finished, this action is finished
        if (actions.isEmpty()) {
            return false;
        }

        //if the current action is running, this action is running
        if (actions.get(0).run(p)) {
            return true;
        } else {
            //if the current action ended, remove it and continue this action
            actions.remove(0);
            //if there are more actions, initialize the next one
            if (!actions.isEmpty()) {
                actions.get(0).init();
            }
            //if an action is finished, run the next action in the sequence
            return run(p);
        }
    }

    @Override
    public void init() {
        //init the first action in the sequence, it is what it will start with
        //this will never be init in the main run(), so it has to be here
        actions.get(0).init();
    }

    @Override
    public void stop(boolean interrupted) {
        for (EventAction act : actions){
            act.stop(interrupted);
        }
    }


}
