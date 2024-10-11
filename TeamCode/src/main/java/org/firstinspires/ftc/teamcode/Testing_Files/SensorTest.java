package org.firstinspires.ftc.teamcode.Testing_Files;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "SensorTest", group = "Testing")
public class SensorTest extends LinearOpMode {

    NormalizedColorSensor colorRange;

    DistanceSensor distanceSensor;

    TouchSensor touchSensor;

    ElapsedTime loopTime;
    @Override
    public void runOpMode() throws InterruptedException {

        colorRange = hardwareMap.get(NormalizedColorSensor.class, "color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;

        loopTime = new ElapsedTime();

        waitForStart();

        while(opModeIsActive()) {

            NormalizedRGBA colors = colorRange.getNormalizedColors();

            telemetry.addData("deviceName", distanceSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", distanceSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", distanceSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", distanceSensor.getDistance(DistanceUnit.INCH)));

            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);

            telemetry.addData("LoopTime", loopTime.milliseconds());
            loopTime.reset();

            telemetry.update();



        }

    }
}
