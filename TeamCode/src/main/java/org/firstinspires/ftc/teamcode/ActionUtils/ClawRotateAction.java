package org.firstinspires.ftc.teamcode.ActionUtils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;

public class ClawRotateAction extends EventAction {

    private CRServo claw;

    private double clawPower;

    public ClawRotateAction(HardwareMap hMap, String name, double power) {
        claw = hMap.get(CRServo.class, name);
        clawPower = power;
    }

    @Override
    public boolean run(CombinedTelemetry t) {
        isRunning = true;

        claw.setPower(clawPower);
        return true;
    }

    @Override
    public void init() {
        claw.setPower(0);
    }

    @Override
    public void stop(boolean interrupted) {
        isRunning = false;
        claw.setPower(0);
    }
}
