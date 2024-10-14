package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

public class FailsafeAction implements EventAction{

    private EventAction runAction, responseAction;
    private FailsafeTrigger failsafeTrigger;

    private boolean triggered, lastTriggered;

    public FailsafeAction(EventAction act, FailsafeTrigger failsafe) {
        runAction = act;
        failsafeTrigger = failsafe;
        responseAction = failsafeTrigger.getAction();

        triggered = false;
        lastTriggered = false;
    }


    @Override
    public boolean run(CombinedTelemetry t) {

        //if a failsafe hasn't been triggered yet, continue looking
        if (!triggered) {
            triggered = failsafeTrigger.triggered();
        }

        //if this was triggered this loop, init the response and interrupt the running action
        if (triggered && !lastTriggered) {
            responseAction.init();
            runAction.stop(true);
        }

        lastTriggered = triggered;

        if (triggered) {

            if (responseAction.run(t)) {
                //once response ends, stop the trigger and reinit the running action
                triggered = false;
                responseAction.stop(false);
                runAction.init();
            }

            return true;
        } else {
            //if the failsafe hasn't been triggered, run the action as normal
            return runAction.run(t);
        }

    }

    @Override
    public void init() {
        runAction.init();
        triggered = false;
        lastTriggered = false;
    }

    @Override
    public void stop(boolean interrupted) {
        if (triggered) {
            responseAction.stop(interrupted);
        } else {
            runAction.stop(interrupted);
        }
    }
}
