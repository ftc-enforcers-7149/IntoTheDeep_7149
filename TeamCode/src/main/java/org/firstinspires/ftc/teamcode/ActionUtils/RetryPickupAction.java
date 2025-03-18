package org.firstinspires.ftc.teamcode.ActionUtils;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.ColorClaw;

public class RetryPickupAction extends EventAction {

    private EventAction pickupAction;
    private ColorClaw colorClaw;
    private ColorClaw.SampleColor wrongColor;

    private boolean isRetry;

    public RetryPickupAction(EventAction pickupAct, ColorClaw claw, ColorClaw.SampleColor wrongColor) {
        pickupAction = pickupAct;
        colorClaw = claw;
        this.wrongColor = wrongColor;

        isRetry = false;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        if (!isRetry) {
            if (colorClaw.getSampleColor() == wrongColor) {
                isRetry = true;
            } else {
                return false;
            }
        }

        isRetry = pickupAction.run(t);

        return true;
    }

    @Override
    public void init() {
        isRetry = false;
    }

    @Override
    public void stop(boolean interrupted) {
        isRetry = false;
    }
}
