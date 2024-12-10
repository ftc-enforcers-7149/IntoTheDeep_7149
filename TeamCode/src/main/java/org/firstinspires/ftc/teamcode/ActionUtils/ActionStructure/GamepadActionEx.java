package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.function.Predicate;

public class GamepadActionEx extends EventAction {

    private ArrayList<EventAction> actions;
    private ArrayList<TriggerStates> triggerStates;
    private ArrayList<Boolean> actionStates;

    private Predicate<? super Gamepad> actionTrigger;
    private Gamepad gamepad;

    private Trigger triggerEx;

    private boolean triggered, lastTriggered;

    private enum TriggerStates {
        WHEN_PRESSED, //runs when pressed, continues until the action ends
        WHILE_HELD, //runs continuously while held, action is stopped when released
        WHEN_RELEASED, //runs when released, continues until action ends
        WHILE_RELEASED, //runs continuously while released, action is stopped when pressed
    }

    //TODO: For enhanced actions, create a class that combines a trigger and an eventAction (or arrayList of each)
    // and have factory classes that add one of these objects to a list
    // and the main class loops through the list of those objs and tests each one
    // using the gamepad
    // AND
    // each type of trigger (whileHeld, whenReleased) needs their own type of method/predicate
    // to be tested using the actual predicates of the actions

    //TODO: Try continuous and non-continuous versions of each while triggerState
    // to prevent/allow repeats of the action

    //TODO: Add multiple conditionals by using generic methods (public <T> void foo(T bar) )
    // with type object references and type predicates, and have the ability to chain them
    // together to have complex triggers
    // Maybe make complex trigger objects and have those as input to constructor


    public GamepadActionEx(Gamepad gpad, Predicate<? super Gamepad> trigger) {
        actionTrigger = trigger;
        gamepad = gpad;

        actions = new ArrayList<EventAction>();
        triggerStates = new ArrayList<TriggerStates>();
        actionStates = new ArrayList<Boolean>();

        triggered = false;
        lastTriggered = false;

    }

    public GamepadActionEx(Trigger trigger) {
        this.triggerEx = trigger;

        actions = new ArrayList<EventAction>();
        triggerStates = new ArrayList<TriggerStates>();
        actionStates = new ArrayList<Boolean>();

        triggered = false;
        lastTriggered = false;

    }


    public GamepadActionEx whenPressed(EventAction action) {
        actions.add(action);
        triggerStates.add(TriggerStates.WHEN_PRESSED);
        actionStates.add(false);

        return this;
    }

    public GamepadActionEx whileHeld(EventAction action) {
        actions.add(action);
        triggerStates.add(TriggerStates.WHILE_HELD);
        actionStates.add(false);
        return this;
    }

    public GamepadActionEx whenReleased(EventAction action) {
        actions.add(action);
        triggerStates.add(TriggerStates.WHEN_RELEASED);
        actionStates.add(false);
        return this;
    }

    public GamepadActionEx whileReleased(EventAction action) {
        actions.add(action);
        triggerStates.add(TriggerStates.WHILE_RELEASED);
        actionStates.add(false);
        return this;
    }

    @Override
    public boolean run(CombinedTelemetry t) {
        isRunning = true;

        //if the action hasn't been triggered, check for the trigger
        if (triggerEx == null) {
            triggered = actionTrigger.test(gamepad);
        } else {
            triggered = triggerEx.isTriggered();
        }

        for (int i = 0; i < actions.size(); i++) {

            switch(triggerStates.get(i)) {


                case WHEN_PRESSED: {
                    //if the action isn't running, check to see if it is triggered
                    if (!actionStates.get(i)) {

                        //if the action was just triggered, init and start running it
                        if (triggered && !lastTriggered) {
                            actions.get(i).init();
                            actionStates.set(i, actions.get(i).run(t));
                        }

                    } else {
                        //if the action is running, continue to run it and stop when finished
                        actionStates.set(i, actions.get(i).run(t));
                    }

                    break;
                }


                case WHILE_HELD: {

                    //if just started to hold down, init the action
                    if (triggered && !lastTriggered) {
                        actions.get(i).init();
                    }

                    //while it is being held down, run the action
                    if (triggered) {
                        actions.get(i).run(t);
                    } else if (lastTriggered) {
                        //when it first ends, stop the action
                        actions.get(i).stop(false);
                    }

                    break;
                }


                case WHEN_RELEASED: {
                    //if the action isn't running, check to see if it is released
                    if (!actionStates.get(i)) {

                        //if the action was just released, init and start running it
                        if (!triggered && lastTriggered) {
                            actions.get(i).init();
                            actionStates.set(i, actions.get(i).run(t));
                        }

                    } else {
                        //if the action is running, continue to run it and stop when finished
                        actionStates.set(i, actions.get(i).run(t));
                    }

                    break;
                }


                case WHILE_RELEASED: {

                    //if just started to release, init the action
                    if (!triggered && lastTriggered) {
                        actions.get(i).init();
                    }

                    //while it is being released, run the action
                    if (!triggered) {
                        actions.get(i).run(t);
                    } else if (!lastTriggered) {
                        //when it first starts, stop the action
                        actions.get(i).stop(false);
                    }

                    break;

                }

            }
        }

        lastTriggered = triggered;

        return true;
    }

    @Override
    public void init() {
        triggered = false;
        lastTriggered = false;

        for (EventAction act : actions) {
            act.init();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        isRunning = false;

        for (EventAction act : actions) {
            act.init();
        }

        triggered = false;
        lastTriggered = false;
    }


}

