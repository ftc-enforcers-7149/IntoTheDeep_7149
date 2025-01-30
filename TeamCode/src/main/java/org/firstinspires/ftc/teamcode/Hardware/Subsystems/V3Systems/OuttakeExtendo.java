package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeExtendo {

    private Servo leftExt, rightExt;
    private double extendoPos;

    public static final double MAX_EXTENSION = 0.2;

    public OuttakeExtendo(OpMode opMode) {
        leftExt = opMode.hardwareMap.get(Servo.class, "leftExt");
        rightExt = opMode.hardwareMap.get(Servo.class, "rightExt");
        extendoPos = 0;
    }

    public void setExtension(double pos) {
        leftExt.setPosition(pos);
        rightExt.setPosition(pos);
        extendoPos = pos;
    }

    public double getExtensionPos(){
        return extendoPos;
    }

}
