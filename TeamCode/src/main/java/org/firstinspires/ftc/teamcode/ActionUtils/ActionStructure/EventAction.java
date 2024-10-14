package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

//TODO: Create an interface for action information,
// have objects that derive from this such as button press objects, sensor information objects.
// Each action's run method gets an input as a derivation of the action info interface

//TODO: add init() to reset or start specific parts of action when it is first run (eg, resetting pid)
//TODO: add stop(boolean interrupt) to either stop the function once it is finished running
// or to interrupt the function and force it to stop in the middle of execution
//Above methods would be especially useful in a teleOp setting where actions are associated with button objects

public interface EventAction {

    /**
     * @param t TelemetryPacket used for communication
     * @return true if the action is still running, false if the action has ended
     */
    public boolean run(CombinedTelemetry t);

    /**
     * Initializes the action for use. Called prior to the action being run
     */
    public void init();

    /**
     * Stops the action from running. Either the action ends naturally or it is interrupted.
     * @param interrupted Dictates whether the action ended normally or it was interrupted.
     */
    public void stop(boolean interrupted);

    //TODO: Should an updateInfo() method be implemented so actions are forced to update
    // any associated information?

}
