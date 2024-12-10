package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;
import org.opencv.core.RotatedRect;

import java.util.List;

public class LLSampleVision implements PeriodicAction {

    private Limelight3A limelight;
    private OpMode opMode;

    public LLSampleVision(OpMode opMode, int pipelineNum) {
        this.opMode = opMode;

        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineNum);

        limelight.start();
    }

    public double[] getResultInfo() {

        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();

    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void startLimelight() {
        limelight.start();
    }

    @Override
    public void periodic() {

    }
}
