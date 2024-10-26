package org.firstinspires.ftc.teamcode.OpenCVPipelines;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EOCV_Pipelines.TesterPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@TeleOp(name = "ProcessorTester")
public class ProcessorTestOpmode extends LinearOpMode {

    TesterPipeline proc;

    @Override
    public void runOpMode() throws InterruptedException {

        proc = new TesterPipeline();

        VisionPortal portal = new VisionPortal.Builder()
                //.addProcessor(proc)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();


        waitForStart();

        if (isStopRequested()){
            return;
        }

        while(opModeIsActive() && !isStopRequested()){
            //FtcDashboard.getInstance().sendImage(proc.getLastFrame());

            telemetry.addData("Yellow Samples", proc.getYellowBoxes());
            telemetry.update();
        }

    }
}
