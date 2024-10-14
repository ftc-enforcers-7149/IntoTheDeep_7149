package org.firstinspires.ftc.teamcode.Testing_Files;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

@Disabled
@Config
@TeleOp(name="Single Hub Test")
public class HubTest extends LinearOpMode {

    DcMotorEx m1, m2, m3, m4;
    Servo s1, s2, s3, s4, s5, s6;
    RevTouchSensor t1, t2, t3, t4, touchSensor2;
    NormalizedColorSensor c1, c2, c3, c4, colorSensor2;
    AnalogInput p1, p2, potentiometer2;

    public static class CameraStreamProc implements VisionProcessor, CameraStreamSource{

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

        public Bitmap getLastFrame(){
            return lastFrame.get();
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m4 = hardwareMap.get(DcMotorEx.class, "m4");

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");
        s6 = hardwareMap.get(Servo.class, "s6");

        t1 = hardwareMap.get(RevTouchSensor.class, "t1");
        t2 = hardwareMap.get(RevTouchSensor.class, "t2");
        t3 = hardwareMap.get(RevTouchSensor.class, "t3");
        t4 = hardwareMap.get(RevTouchSensor.class, "t4");

        c1 = hardwareMap.get(NormalizedColorSensor.class, "c1");
        c2 = hardwareMap.get(NormalizedColorSensor.class, "c2");
        c3 = hardwareMap.get(NormalizedColorSensor.class, "c3");
        c4 = hardwareMap.get(NormalizedColorSensor.class, "c4");

        p1 = hardwareMap.get(AnalogInput.class, "p1");
        p2 = hardwareMap.get(AnalogInput.class, "p2");

        CameraStreamProc processor = new CameraStreamProc();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        portal.setProcessorEnabled(processor, true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (opModeIsActive() && !isStopRequested()){

            FtcDashboard.getInstance().sendImage(processor.getLastFrame());

            m1.setPower(gamepad1.left_stick_y);
            telemetry.addData("Motor 1",m1.getCurrentPosition());
            m2.setPower(gamepad1.left_stick_y);
            telemetry.addData("Motor 2",m2.getCurrentPosition());
            m3.setPower(gamepad1.left_stick_y);
            telemetry.addData("Motor 3",m3.getCurrentPosition());
            m4.setPower(gamepad1.left_stick_y);
            telemetry.addData("Motor 4",m4.getCurrentPosition());

            s1.setPosition(gamepad1.left_trigger);
            telemetry.addData("Servo 1", s1.getPosition());
            s2.setPosition(gamepad1.left_trigger);
            telemetry.addData("Servo 2", s2.getPosition());
            s3.setPosition(gamepad1.left_trigger);
            telemetry.addData("Servo 3", s3.getPosition());
            s4.setPosition(gamepad1.left_trigger);
            telemetry.addData("Servo 4", s4.getPosition());
            s5.setPosition(gamepad1.left_trigger);
            telemetry.addData("Servo 5", s5.getPosition());
            s6.setPosition(gamepad1.left_trigger);
            telemetry.addData("Servo 6", s6.getPosition());


            telemetry.addData("Touch 1", t1.isPressed());
            telemetry.addData("Touch 2", t2.isPressed());
            telemetry.addData("Touch 3", t3.isPressed());
            telemetry.addData("Touch 4", t4.isPressed());


            telemetry.addData("Potentiometer 1", p1.getVoltage());
            telemetry.addData("Potentiometer 2", p2.getVoltage());

            NormalizedRGBA colors = c1.getNormalizedColors();
            telemetry.addData("Red", colors.red);
            telemetry.addData("Green", colors.green);
            telemetry.addData("Blue", colors.blue);
            telemetry.addData("Alpha", colors.alpha);
            colors = c2.getNormalizedColors();
            telemetry.addData("Red", colors.red);
            telemetry.addData("Green", colors.green);
            telemetry.addData("Blue", colors.blue);
            telemetry.addData("Alpha", colors.alpha);
            colors = c3.getNormalizedColors();
            telemetry.addData("Red", colors.red);
            telemetry.addData("Green", colors.green);
            telemetry.addData("Blue", colors.blue);
            telemetry.addData("Alpha", colors.alpha);
            colors = c4.getNormalizedColors();
            telemetry.addData("Red", colors.red);
            telemetry.addData("Green", colors.green);
            telemetry.addData("Blue", colors.blue);
            telemetry.addData("Alpha", colors.alpha);

            if (c1 instanceof DistanceSensor){
                double distance = ((DistanceSensor) c1).getDistance(DistanceUnit.MM);
                telemetry.addData("Distance 1", distance);
            }
            if (c2 instanceof DistanceSensor){
                double distance = ((DistanceSensor) c2).getDistance(DistanceUnit.MM);
                telemetry.addData("Distance 2", distance);
            }
            if (c3 instanceof DistanceSensor){
                double distance = ((DistanceSensor) c3).getDistance(DistanceUnit.MM);
                telemetry.addData("Distance 3", distance);
            }
            if (c4 instanceof DistanceSensor){
                double distance = ((DistanceSensor) c4).getDistance(DistanceUnit.MM);
                telemetry.addData("Distance 4", distance);
            }

            telemetry.update();


        }


    }
}
