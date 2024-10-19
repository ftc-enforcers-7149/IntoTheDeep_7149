package org.firstinspires.ftc.teamcode.ActionUtils;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;

public class TwinnedAction extends EventAction {

    private EventAction action1, action2;

    public TwinnedAction(EventAction ac1, EventAction ac2) {
        action1 = ac1;
        action2 = ac2;
    }


    @Override
    public boolean run(CombinedTelemetry t) {
        isRunning = true;

        //run the two actions in order, then return the status of the first
        boolean ended = action1.run(t);
        action2.run(t);

        if (ended) {
            action1.stop(false);
            action2.stop(false);
        }

        return ended;
    }

    @Override
    public void init() {
        action1.init();
        action2.init();
    }

    @Override
    public void stop(boolean interrupted) {
        isRunning = false;
        action1.stop(interrupted);
        action2.stop(interrupted);
    }
}
