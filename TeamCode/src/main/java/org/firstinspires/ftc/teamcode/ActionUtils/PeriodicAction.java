package org.firstinspires.ftc.teamcode.ActionUtils;

/**
 * Implemented by actions or hardware that is constantly run during action execution.
 * PeriodicActions are attached to an ActionManager prior to execution.
 */
public interface PeriodicAction {

    /**
     * Runs periodically each loop during action execution.
     */
    public void periodic();

}
