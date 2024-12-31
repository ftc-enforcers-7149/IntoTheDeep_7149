package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends EventAction {

    private int timeMs;

    private boolean started;
    private double interruptedTime;

    private ElapsedTime timer;

    public WaitAction(int ms) {
        timeMs = ms;

        started = false;
        interruptedTime = 0;

        timer = new ElapsedTime();
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        isRunning = true;

        if (!started) {
            timer.reset();
            started = true;
        }

        t.getTelemetry().addData("WaitAction", timeMs + "  " + timer.milliseconds());

        if (!((timer.milliseconds() + interruptedTime) < timeMs)) {
            stop(false);
            return false;
        }

        return true;
    }

    @Override
    public void init() {
        started = false;
        interruptedTime = 0;
    }

    @Override
    public void stop(boolean interrupted) {

        isRunning = false;

        //save the time at which the action was interrupted
        if (interrupted) {
            interruptedTime = timer.milliseconds();
        }

        started = false;
    }

}
