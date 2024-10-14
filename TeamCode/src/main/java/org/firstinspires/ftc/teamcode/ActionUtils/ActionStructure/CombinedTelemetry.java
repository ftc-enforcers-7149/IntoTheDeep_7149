package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CombinedTelemetry {

    private Telemetry telemetry;
    private TelemetryPacket packet;

    public CombinedTelemetry(Telemetry t, TelemetryPacket p) {
        telemetry = t;
        packet = p;
    }

    public TelemetryPacket getPacket() {
        return packet;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
}
