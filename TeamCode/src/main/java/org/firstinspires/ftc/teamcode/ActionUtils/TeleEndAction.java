package org.firstinspires.ftc.teamcode.ActionUtils;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.GamepadAction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TeleEndAction extends EventAction {

    private ArrayList<GamepadAction> secondaryActions;
    private GamepadAction firstAction;

    private boolean startedRunning, lastStartedRunning;

    public TeleEndAction(GamepadAction action1, List<GamepadAction> actionList) {
        firstAction = action1;
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        secondaryActions = new ArrayList<GamepadAction>((List<GamepadAction>) actionList);

        startedRunning = false;
        lastStartedRunning = false;
    }

    public TeleEndAction(GamepadAction action1, GamepadAction... actions) {
        this(action1, Arrays.asList(actions));
    }


    @Override
    public boolean run(CombinedTelemetry t) {
        isRunning = true;

        //check when the first action is triggered
        boolean actionRanAndFinished = firstAction.run(t);
        if (!startedRunning) {
            startedRunning = firstAction.isTriggered();
        }

        if (startedRunning && !lastStartedRunning) {
            //stop all of the other actions and let the other one continue
            for (EventAction action : secondaryActions) {
                action.stop(false);
            }

        } else {
            //run the other actions in parallel
            for (EventAction action : secondaryActions) {
                action.run(t);
            }
        }

        lastStartedRunning = startedRunning;

        return actionRanAndFinished;
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
