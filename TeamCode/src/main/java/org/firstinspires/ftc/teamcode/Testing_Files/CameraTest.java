package org.firstinspires.ftc.teamcode.Testing_Files;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.concurrent.atomic.AtomicReference;

@Disabled
@TeleOp(name="CameraTest")
public class CameraTest extends LinearOpMode {

    public static class CameraStreamProc implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        public Bitmap getLastFrame() {
            return lastFrame.get();
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }

    }


    @Override
    public void runOpMode() throws InterruptedException {

        boolean gotSnapshot = false;
        int loops = 0;

        CameraStreamProc processor = new CameraStreamProc();

        //TesterPipeline processor = new TesterPipeline();


        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320,240))
                .addProcessor(processor)
                .build();
        portal.setProcessorEnabled(processor, true);

        waitForStart();

        if(isStopRequested()){
            return;
        }

        while(opModeIsActive() && !isStopRequested()){

            if (gamepad1.cross) {
                portal.resumeStreaming();
                gotSnapshot = true;
            }

            if (gamepad1.triangle) {
                portal.stopStreaming();
                gotSnapshot = false;
            }

            VisionPortal.CameraState state = portal.getCameraState();

            if (state == VisionPortal.CameraState.STARTING_STREAM) {
                loops++;
            }





            WebcamName cam = hardwareMap.get(WebcamName.class, "Cam");
            OpenCvCamera cvCam = OpenCvCameraFactory.getInstance().createWebcam(cam);



            FtcEventLoop loop = new FtcEventLoop(null, null, null, null);
            OpModeManagerImpl manager = loop.getOpModeManager();







            FtcDashboard.getInstance().sendImage(processor.getLastFrame());

            telemetry.addData("Camera State", state.name());
            telemetry.addData("Loops to Start Stream", loops);
            telemetry.addData("Snapshot", gotSnapshot);
            telemetry.update();



        }

    }
}
