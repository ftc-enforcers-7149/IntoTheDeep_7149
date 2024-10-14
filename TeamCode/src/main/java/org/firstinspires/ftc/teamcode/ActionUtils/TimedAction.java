package org.firstinspires.ftc.teamcode.ActionUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;

public class TimedAction implements EventAction {

    private EventAction action;
    private ElapsedTime timer;
    private int msTimeLimit;

    private boolean started;

    /**
     * Runs an action for a specified amount of time before terminating it.
     * @param act Action to be ran
     * @param msTime Amount of time to run the action (milliseconds)
     */
    public TimedAction(EventAction act, int msTime) {
        action = act;
        timer = new ElapsedTime();
        msTimeLimit = msTime;
        started = false;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        if (!started) {
            started = true;
            timer.reset();
        }

        action.run(t);

        if (timer.milliseconds() > msTimeLimit) {
            this.stop(false);
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void init() {
        action.init();
        started = false;
        timer.reset();
    }

    @Override
    public void stop(boolean interrupted) {
        started = false;
        action.stop(interrupted);
        timer.reset();
    }
}
