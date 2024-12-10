package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import java.util.ArrayList;
import java.util.function.Predicate;

public class Trigger<T> {

    private T triggerObj;
    private Predicate<T> trigger;

    private Trigger extraTrigger;
    private TriggerOperators operator;

    private enum TriggerOperators{
        AND,
        OR,
    }

    public Trigger(T triggerObj, Predicate<T> trigger) {
        this.triggerObj = triggerObj;
        this.trigger = trigger;
    }

    //TODO: Change the operators so that you can chain more than one together on a single action


    public Trigger and(Trigger trigger) {
        extraTrigger = trigger;
        operator = TriggerOperators.AND;
        return this;
    }

    public Trigger or(Trigger trigger) {
        extraTrigger = trigger;
        operator = TriggerOperators.OR;
        return this;
    }

    public <E> Trigger and(E triggerObj, Predicate<E> trigger) {
        extraTrigger = new Trigger<E>(triggerObj, trigger);
        operator = TriggerOperators.AND;
        return this;
    }

    public <E> Trigger or(E triggerObj, Predicate<E> trigger) {
        extraTrigger = new Trigger<E>(triggerObj, trigger);
        operator = TriggerOperators.OR;
        return this;
    }

    public boolean isTriggered() {

        if (extraTrigger != null) {

            switch(operator) {

                case AND: {
                    return trigger.test(triggerObj) && extraTrigger.isTriggered();
                }

                case OR: {
                    return trigger.test(triggerObj) || extraTrigger.isTriggered();
                }

                default: {
                    return false;
                }

            }

        } else {

            return trigger.test(triggerObj);

        }

    }
}
