package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import java.util.ArrayList;
import java.util.Arrays;

public class ConditionalAction extends EventAction {

    public interface ChoiceLambda<T>{
        public T get();
    }

    private ArrayList<EventAction> actions;
    private ChoiceLambda<Boolean> booleanChoiceLambda;
    private ChoiceLambda<Integer> integerChoiceLambda;
    private EventAction chosenAction;

    private boolean multiChoice;
    private boolean chosen;


    public ConditionalAction(ChoiceLambda<Integer> choiceLambda, EventAction...acts) {
        actions = new ArrayList<>(Arrays.asList(acts));
        integerChoiceLambda = choiceLambda;

        multiChoice = true;
        booleanChoiceLambda = null;

        chosen = false;
    }

    public ConditionalAction(ChoiceLambda<Boolean> choiceLambda, EventAction act1, EventAction act2) {
        actions = new ArrayList<>(Arrays.asList(act1, act2));
        booleanChoiceLambda = choiceLambda;

        multiChoice = false;
        integerChoiceLambda = null;

        chosen = false;
    }

    public ConditionalAction(ChoiceLambda<Boolean> choiceLambda, EventAction act1) {
        actions = new ArrayList<>(Arrays.asList(act1));
        booleanChoiceLambda = choiceLambda;

        multiChoice = false;
        integerChoiceLambda = null;

        chosen = false;
    }


    @Override
    public boolean run(CombinedTelemetry t) {

        if (!chosen) {

            if (multiChoice) {
                chosenAction = actions.get(integerChoiceLambda.get());

            } else {

                int index;
                if (booleanChoiceLambda.get()) {
                    index = 0;
                } else {
                    if (actions.size() > 1) {
                        index = 1;
                    } else {
                        return false;
                    }
                }

                chosenAction = actions.get(index);
            }

            chosenAction.init();
            chosen = true;
        }

        return chosenAction.run(t);
    }

    @Override
    public void init() {
        chosen = false;
    }

    @Override
    public void stop(boolean interrupted) {
        chosenAction.stop(interrupted);
    }
}
