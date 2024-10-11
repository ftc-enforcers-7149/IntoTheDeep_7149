package org.firstinspires.ftc.teamcode.Testing_Files;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "sonarTest")

public class SonarTesting extends LinearOpMode {
    AnalogInput sonarSensor;

    @Override
    public void runOpMode() {
        sonarSensor = hardwareMap.get(AnalogInput.class, "sonar");

        waitForStart();

        while (opModeIsActive()) {

            double voltage = sonarSensor.getVoltage();

            // Convert the voltage to distance (based on sensor specifications)
            double distance = voltageToDistance(voltage);

            telemetry.addData("Distance (in)", distance);
            telemetry.update();

        }

    }

    private double voltageToDistance(double voltage) {
        // Replace with the conversion formula for your specific sonar sensor
        // For example, if the sensor outputs 0-5V for 0-200 cm range, the formula might be:
        return (voltage / 3.3) * 204.86;  // Example formula for scaling
    }
}
