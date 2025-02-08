package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ActionManager {

    public interface TelemetryUpdater{
        public void update(Telemetry t);
    }

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

    public ActionManager(OpMode opMode) {
        this(opMode.telemetry, opMode.hardwareMap);
    }

    public void attachPeriodicActions(List<PeriodicAction> periodic) {
        periodicActions = new ArrayList<PeriodicAction>((List<PeriodicAction>) periodic);
    }

    public void attachPeriodicActions(PeriodicAction...periodic) {
        attachPeriodicActions(Arrays.asList(periodic));
    }


    public void runActionManager(EventAction a, TelemetryUpdater telemetryUpdate) {

        FtcDashboard dash = FtcDashboard.getInstance();

        boolean b = true;

        a.init(); //init this action before it is ran

        while (b && !Thread.currentThread().isInterrupted()) {

            //reset stale readings for bulk reads
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //update hardware linked to the action manager
            for (PeriodicAction perAct : periodicActions) {
                perAct.periodic();
            }

            CombinedTelemetry t = new CombinedTelemetry(telemetry, new TelemetryPacket());
            b = a.run(t);

            dash.sendTelemetryPacket(t.getPacket());
            telemetry.addLine("");
            telemetry.addData("LoopTime(ms)", loopTime.milliseconds());
            telemetry.addLine("");
            telemetryUpdate.update(telemetry);

            loopTime.reset();
            telemetry.update();
        }

    }

}
