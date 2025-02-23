package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;
import org.opencv.core.RotatedRect;

import java.util.List;

public class LLSampleVision implements PeriodicAction {

    private Limelight3A limelight;
    private OpMode opMode;
    private Telemetry tele;

    public LLSampleVision(OpMode opMode, int pipelineNum) {
        this.opMode = opMode;
        tele = opMode.telemetry;

        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineNum);

        limelight.setPollRateHz(100);
        limelight.start();
    }

    public double[] getResultInfo() {

        LLResult result = limelight.getLatestResult();


        if (result == null) {
            tele.addData("Null", "LLResult");
            return new double[8];
        }

        tele.addData("Running", limelight.isRunning());
        tele.addData("Info Status", result.isValid());
        tele.addData("Pipeline", result.getPipelineIndex() + " Type: " + result.getPipelineType() + " |");

        return result.getPythonOutput();

    }

    public double getServoPos() {

        return (getResultInfo()[0] + Math.PI/2) / Math.PI;
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void closeLimelight() {
        limelight.close();
    }

    public void startLimelight() {
        limelight.start();
    }

    public boolean getConnectionStatus() {
        return limelight.isConnected();
    }

    public boolean getRunningStatus() {
        return limelight.isRunning();
    }

    @Override
    public void periodic() {

    }
}
