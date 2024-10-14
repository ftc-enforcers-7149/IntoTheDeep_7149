package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

//With this interfacing, a failsafe could technically be triggered while another failsafe is running

public interface FailsafeTrigger{

    /**
     * Updates the status of this failsafe.
     * @return The status of the failsafe.
     */
    public boolean triggered();

    /**
     * @return This failsafe's response action.
     */
    public EventAction getAction();

}