package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristRotateAction extends EventAction{

    private Servo wrist;
    private HardwareMap hMap;
    private double position;

    public WristRotateAction(double pos, HardwareMap map, String name) {
        hMap = map;
        wrist = hMap.get(Servo.class, name);
        position = pos;
        wrist.setPosition(0.5);
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        wrist.setPosition(position);

        return false;
    }

    @Override
    public void init() {

    }

    @Override
    public void stop(boolean interrupted) {

    }
}
