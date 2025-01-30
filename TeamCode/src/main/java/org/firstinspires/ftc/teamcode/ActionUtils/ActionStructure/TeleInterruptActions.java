package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

public class TeleInterruptActions<E> extends EventAction{

    private ArrayList<GamepadAction> runActions;
    private EventAction mainAction;
    private EventAction runningAction;

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

//        t.getTelemetry().addData("Stopped", stopped);
//        t.getTelemetry().addData("PRev", prevStopped);

        //if the main action isn't stopped, continue running it
        AllActions: if (!stopped) {
            mainAction.run(t);

            //go through all actions to see if one is triggered
            for (GamepadAction act : runActions) {
                act.run(t);
                //if this action is triggered, stop the first one
                if (act.isTriggered()) {
                    stopped = true;
                    act.init(); //reset the trigger on this action so it doesn't stay triggered forever
                    mainAction.stop(true);
                    //get the base action from the gamepad action and init it
                    runningAction = act.getAction();
                    runningAction.init();
                    break AllActions;
                }
            }
        }

        //if the main action is stopped, run the secondary action
        if (stopped) {
            stopped = runningAction.run(t);
            //stopped = runningAction.isTriggered();

            //if the trigger is triggered, stop the secondary and restart the main
            if (actionTrigger.test(triggerObj)) {
                stopped = false;
                runningAction.stop(true);
                mainAction.init();
            }
        }

        //if this secondary action just stopped, restart the main
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
