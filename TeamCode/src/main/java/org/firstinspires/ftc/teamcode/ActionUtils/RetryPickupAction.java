package org.firstinspires.ftc.teamcode.ActionUtils;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.ColorClaw;

public class RetryPickupAction extends EventAction {

    private SequentialAction pickupAction, backUp;
    private ColorClaw colorClaw;
    private ColorClaw.SampleColor wrongColor;

    private boolean isRetry;

    public RetryPickupAction(SequentialAction pickupAct, ColorClaw claw, ColorClaw.SampleColor wrongColor) {
        pickupAction = pickupAct;
        backUp = pickupAction.copy();
        colorClaw = claw;
        this.wrongColor = wrongColor;

        isRetry = false;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        if (!isRetry) {
            if (colorClaw.getSampleColor() == wrongColor || colorClaw.getSampleColor() == ColorClaw.SampleColor.NONE) {
                isRetry = true;
                pickupAction = backUp.copy();
                pickupAction.init();
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
