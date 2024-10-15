package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ActionManager {

    ArrayList<PeriodicAction> periodicActions;
    Telemetry telemetry;

    ElapsedTime loopTime;

    public ActionManager(Telemetry tele) {
        telemetry = tele;
        periodicActions = new ArrayList<PeriodicAction>();
        loopTime = new ElapsedTime();
    }

    public void attachPeriodicActions(List<PeriodicAction> periodic) {
        periodicActions = new ArrayList<PeriodicAction>((List<PeriodicAction>) periodic);
    }

    public void attachPeriodicActions(PeriodicAction...periodic) {
        attachPeriodicActions(Arrays.asList(periodic));
    }


    public void runActionManager(EventAction a) {

        //TODO: Make an action that takes in two EventActions and ends the second one once
        // the first one ends. In this case, the second action continuously runs until the
        // first action ends

        FtcDashboard dash = FtcDashboard.getInstance();

        boolean b = true;

        a.init(); //init this action before it is ran

        while (b && !Thread.currentThread().isInterrupted()) {

            for (PeriodicAction perAct : periodicActions) {
                perAct.periodic();
            }

            CombinedTelemetry t = new CombinedTelemetry(telemetry, new TelemetryPacket());
            b = a.run(t);

            dash.sendTelemetryPacket(t.getPacket());
            telemetry.addLine("");
            telemetry.addData("LoopTime(ms)", loopTime.milliseconds());
            loopTime.reset();
            telemetry.update();
        }

        //TODO: Have periodic actions running in the back to update actionInfo objects associated with actions
    }

}
