package org.firstinspires.ftc.teamcode.Testing_Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorSensor", group="Testers")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            double R = sensor.red();
            double G = sensor.green();
            double B = sensor.blue();

            telemetry.addData("R", R);
            telemetry.addData("G", G);
            telemetry.addData("B", B);

            if (sensor instanceof DistanceSensor) {

                DistanceSensor distSensor = (DistanceSensor) sensor;

                telemetry.addData("Distance", distSensor.getDistance(DistanceUnit.MM));

            }

            String sampleType;

            if ((B / R) > 1.4) {
                sampleType = "Blue";
            } else if ((R / G) > 1.8) {
                sampleType = "Red";
            } else {
                sampleType = "Yellow";
            }

            telemetry.addData("Sample Color", sampleType);

            telemetry.update();

        }

    }
}
