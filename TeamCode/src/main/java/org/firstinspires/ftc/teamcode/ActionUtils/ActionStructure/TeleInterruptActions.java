package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

public class TeleInterruptActions<E> extends EventAction{

    private ArrayList<GamepadAction> runActions;
    private EventAction mainAction;
    private GamepadAction runningAction;

    private Predicate<? super E> actionTrigger;
    private E triggerObj;

    private boolean stopped, prevStopped;

    public TeleInterruptActions(EventAction main, E trigObj, Predicate<? super E> trigger, List<GamepadAction> actionList) {
        //ensure we are given an arrayList, or else we get exceptions when removing actions from list
        runActions = new ArrayList<GamepadAction>((List<GamepadAction>) actionList);
        mainAction = main;

        actionTrigger = trigger;
        triggerObj = trigObj;

        stopped = false;
        prevStopped = false;
    }

    public TeleInterruptActions(EventAction main, E trigObj, Predicate<? super E> trigger, GamepadAction...actions) {
        this(main, trigObj, trigger, Arrays.asList(actions));
    }


    @Override
    public boolean run(CombinedTelemetry t) {

        isRunning = true;

        AllActions: if (!stopped) {
            mainAction.run(t);

            for (GamepadAction act : runActions) {
                act.run(t);
                if (act.isTriggered()) {
                    stopped = true;
                    mainAction.stop(true);
                    runningAction = act;
                    runningAction.init();
                    break AllActions;
                }
            }
        }

        if (stopped && !prevStopped) {
            runningAction.run(t);
            stopped = runningAction.isTriggered();

            if (actionTrigger.test(triggerObj)) {
                stopped = false;
                runningAction.stop(true);
                mainAction.init();
            }
        }

        if (!stopped && prevStopped) {
            runningAction.stop(false);
        }

        prevStopped = stopped;

        return false;
    }

    @Override
    public void init() {
        mainAction.init();
        for (EventAction action : runActions) {
            action.init();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        isRunning = false;
        if (stopped) {
            runningAction.stop(interrupted);
        } else {
            mainAction.stop(interrupted);
        }

    }

    public boolean isTriggered(){
        return stopped;
    }

}
