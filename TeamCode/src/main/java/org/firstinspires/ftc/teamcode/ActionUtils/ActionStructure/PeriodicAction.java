package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

/**
 * Implemented by actions or hardware systems that are constantly run during action execution.
 * PeriodicActions are attached to an ActionManager prior to execution.
 */
public interface PeriodicAction {

    /**
     * Runs periodically each loop during action execution.
     */
    public void periodic();

}
