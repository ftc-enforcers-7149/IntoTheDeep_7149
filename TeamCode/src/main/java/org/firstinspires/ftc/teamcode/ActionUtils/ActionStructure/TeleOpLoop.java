package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TeleOpLoop extends EventAction {

    private ArrayList<EventAction> actions;

    public TeleOpLoop(List<EventAction> actionList) {
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        actions = new ArrayList<EventAction>((List<EventAction>) actionList);
    }

    public TeleOpLoop(EventAction...actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(CombinedTelemetry t) {
        //run all of the actions in the list
        for (EventAction act : actions) {
            act.run(t);
        }
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
}
