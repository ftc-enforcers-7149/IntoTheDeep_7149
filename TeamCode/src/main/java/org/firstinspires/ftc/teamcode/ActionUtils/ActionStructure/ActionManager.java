package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ActionManager {

    private ArrayList<PeriodicAction> periodicActions;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private ArrayList<LynxModule> allHubs;

    private ElapsedTime loopTime;

    public ActionManager(Telemetry tele, HardwareMap hMap) {
        telemetry = tele;
        periodicActions = new ArrayList<PeriodicAction>();
        loopTime = new ElapsedTime();

        hardwareMap = hMap;

        allHubs = new ArrayList<LynxModule>(hardwareMap.getAll(LynxModule.class));

        //set bulk read mode to manual for hubs
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void attachPeriodicActions(List<PeriodicAction> periodic) {
        periodicActions = new ArrayList<PeriodicAction>((List<PeriodicAction>) periodic);
    }

    public void attachPeriodicActions(PeriodicAction...periodic) {
        attachPeriodicActions(Arrays.asList(periodic));
    }


    public void runActionManager(EventAction a) {


        //TODO: Make gamepad triggered actions that take in a gamepad and a gamepad predicate
        // The predicate specifies the button/trigger that leads to a boolean as to whether
        // the action is triggered or not
        // for these actions, make an abstract class that implements EventAction
        // Abstract class should have a running boolean, a predicate input, and an action input

        //TODO: Should an isRunning method be implemented for EventActions?
        // Could be useful during TeleOp to not be running interfering actions


        FtcDashboard dash = FtcDashboard.getInstance();

        boolean b = true;

        a.init(); //init this action before it is ran

        while (b && !Thread.currentThread().isInterrupted()) {

            //reset stale readings for bulk reads
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

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
