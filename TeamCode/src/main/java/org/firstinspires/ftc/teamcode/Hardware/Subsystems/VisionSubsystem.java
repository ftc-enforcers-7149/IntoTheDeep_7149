package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;
import org.firstinspires.ftc.teamcode.Hardware.VisionProcessors.SampleDetector;
import org.firstinspires.ftc.vision.VisionPortal;

public class VisionSubsystem implements PeriodicAction {

    private VisionPortal visionPortal;
    private SampleDetector detector;
    private OpMode opMode;

    public VisionSubsystem(OpMode mode) {
        opMode = mode;

        detector = new SampleDetector();
        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(detector)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        visionPortal.setProcessorEnabled(detector, true);

    }

    public SampleDetector getDetector() {
        return detector;
    }

    public VisionPortal getPortal() {
        return visionPortal;
    }

    @Override
    public void periodic() {

    }

}
