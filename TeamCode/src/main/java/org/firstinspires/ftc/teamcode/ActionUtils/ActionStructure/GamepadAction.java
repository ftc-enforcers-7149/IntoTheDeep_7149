package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Comparator;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class GamepadAction extends EventAction {

    private EventAction action;
    private Predicate<? super Gamepad> actionTrigger;

    private Gamepad gamepad;

    private boolean triggered, lastTriggered;


    public GamepadAction(EventAction act, Gamepad gpad, Predicate<? super Gamepad> trigger) {
        action = act;
        actionTrigger = trigger;

        gamepad = gpad;
        triggered = false;
        lastTriggered = false;
    }

    @Override
    public boolean run(CombinedTelemetry t) {
        isRunning = true;

        //if the action hasn't been triggered, check for the trigger
        if (!triggered) {
            triggered = actionTrigger.test(gamepad);
        }

        //if the action was just triggered, init the action
        if (triggered && !lastTriggered) {
            action.init();
        }

        //if the action is triggered, and the action has stopped, it is no longer triggered
        if (triggered) {
            if (!action.run(t)) {
                triggered = false;
            }
        }

        //if the action just ended, stop it
        if (lastTriggered && !triggered) {
            action.stop(false);
            lastTriggered = triggered;
            return false;
        }

        lastTriggered = triggered;

        return true;
    }

    @Override
    public void init() {
        triggered = false;
        lastTriggered = false;
    }

    @Override
    public void stop(boolean interrupted) {
        isRunning = false;

        if (triggered) {
            action.stop(interrupted);
        }

        triggered = false;
        lastTriggered = false;
    }


    public boolean isTriggered() {
        return triggered;
    }
}

