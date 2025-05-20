package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorClaw {

    private ColorSensor sensor;
    private SampleColor sampleColor;
    private double distance;

    public enum SampleColor{
        YELLOW, RED, BLUE, NONE;
    }

    public ColorClaw(OpMode opMode) {
        sensor = opMode.hardwareMap.get(ColorSensor.class, "colorSensor");
        sampleColor = SampleColor.NONE;
    }

    public SampleColor getSampleColor() {

        double R = sensor.red();
        double G = sensor.green();
        double B = sensor.blue();
        double dist = 1000;

        if (sensor instanceof DistanceSensor) {
            dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.MM);
        }

        if (dist < 18) {
            if ((B / R) > 1.4) {
                sampleColor = SampleColor.BLUE;
            } else if ((R / G) > 1.8) {
                sampleColor = SampleColor.RED;
            } else {
                sampleColor = SampleColor.YELLOW;
            }
        } else {
            sampleColor = SampleColor.NONE;
        }

        return sampleColor;
    }

    public double getDistance(DistanceUnit unit) {

        if (sensor instanceof DistanceSensor) {
            return ((DistanceSensor) sensor).getDistance(unit);
        }

        return 999;
    }


}
