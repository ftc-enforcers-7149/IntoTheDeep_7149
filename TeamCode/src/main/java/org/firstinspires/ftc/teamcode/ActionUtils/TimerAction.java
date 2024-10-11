package org.firstinspires.ftc.teamcode.ActionUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TimerAction implements EventAction {

    private String text;
    private int timeMs;

    private boolean started;

    private ElapsedTime timer;
    private Telemetry telemetry;

    public TimerAction(int ms, String txt, Telemetry tele) {
        text = txt;
        timer = new ElapsedTime();
        timeMs = ms;
        started = false;
        telemetry = tele;
    }

    @Override
    public boolean run(TelemetryPacket p) {

        if (!started) {
            timer.reset();
            started = true;
        }

        telemetry.addData("TesterAction", text);
        return timer.milliseconds() < timeMs;
    }

    @Override
    public void init() {
        started = false;
        telemetry.addData("Initialized", text);
    }

    @Override
    public void stop(boolean interrupted) {
        started = false;
        telemetry.addData("Stopped", interrupted + " " + text);
    }

}
