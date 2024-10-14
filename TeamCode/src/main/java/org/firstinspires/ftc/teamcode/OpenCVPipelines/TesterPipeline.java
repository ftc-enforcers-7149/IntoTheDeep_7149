package org.firstinspires.ftc.teamcode.OpenCVPipelines;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

//TODO: Make an abstract processor base class to extend all processors from, should contain hsv values for specific colors and useful methods

//@Config
public class TesterPipeline extends OpenCvPipeline{

    public static Scalar lowerRed = new Scalar(215,50,50);
    public static Scalar upperRed = new Scalar(255,200,225);
    public static Scalar lowerBlue = new Scalar(145,50,50);
    public static Scalar upperBlue = new Scalar(175,200,225);
    public static Scalar lowerYellow = new Scalar(25,110,200); //(10,95,95);
    public static Scalar upperYellow = new Scalar(45,255,255);//(45,255,255);

    public Scalar meanContourValue = new Scalar(0,0,0);

    /*--------------HSV COLORS---------------------------

    FTC Blue: H 145-175  S 50 - 200  V 50-225

    FTC Red: H 215-255  S 50 - 200  V 50-225

    FTC Yellow: H 10-45  S 50 - 200  V 50-225

     --------------------------------------------------*/


    public int contourNum = 0;
    public double contourX = 0;
    public double contourY = 0;
    public double contourHead = 0;
    public double averageBrightness = 0;
    public int minContourArea = 150;

    Mat colorConvert = new Mat();

    Mat redRange = new Mat();
    Mat blueRange = new Mat();
    Mat yellowRange = new Mat();

    public static int kernalSize = 10;

    //Mat morphKernelErode = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(15,15));
    Mat brightness = new Mat();

    private AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public ArrayList<MatOfPoint> redContours = new ArrayList<MatOfPoint>();
    public ArrayList<MatOfPoint> blueContours = new ArrayList<MatOfPoint>();
    public ArrayList<MatOfPoint> yellowContours = new ArrayList<MatOfPoint>();

    public ArrayList<Rect> redBoundingBoxes = new ArrayList<Rect>();
    public ArrayList<Rect> blueBoundingBoxes = new ArrayList<Rect>();
    public ArrayList<Rect> yellowBoundingBoxes = new ArrayList<Rect>();

    public ArrayList<Double> sampleScores = new ArrayList<Double>();

    public ArrayList<RotatedRect> yellowRotBoundingBoxes = new ArrayList<RotatedRect>();

    RotatedRect bestBox;

    DecimalFormat decForm2 = new DecimalFormat("#.00");
    DecimalFormat decForm1 = new DecimalFormat("#.0");

    Point clawPosition = new Point(180, 240);
    public static double angleScoreCoefficient = 1.7;
    public static double xOffsetCoefficient = 1.5;

    //TODO: Break this processor into multiple ones (blue, yellow, and red) to reduce the processing time
    //eg. Based on alliance color chosen through auto, a yellow and alliance color specific processor
    //can be used during teleop, or one processor for each specific color can be used
    @Override
    public Mat processFrame(Mat frame) {

        redContours.clear();
        blueContours.clear();
        yellowContours.clear();
        redBoundingBoxes.clear();
        blueBoundingBoxes.clear();
        yellowBoundingBoxes.clear();
        yellowRotBoundingBoxes.clear();
        sampleScores.clear();

        //get an hls mat to calculate the average brightness
        Imgproc.cvtColor(frame, brightness, Imgproc.COLOR_RGB2HLS);

        //blur the frame to smooth it out and remove imperfections
        //Imgproc.GaussianBlur(frame, frame, new Size(7,7), 0);

        //convert frame to hsv for better lighting invariance
        Imgproc.cvtColor(frame, colorConvert, Imgproc.COLOR_RGB2HLS_FULL);

        //calculate the average brightness of the frame
        //averageBrightness = Core.mean(brightness).val[1];

        //Core.inRange(colorConvert, lowerRed, upperRed, redRange);
        //Core.inRange(colorConvert, lowerBlue, upperBlue, blueRange);
        Core.inRange(colorConvert, lowerYellow, upperYellow, yellowRange);

        //Imgproc.morphologyEx(yellowRange, yellowRange, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(10,10)));
        //Imgproc.morphologyEx(yellowRange, yellowRange, Imgproc.MORPH_CLOSE, morphKernel);

        //TODO: Make the kernal size adaptive based on :
        // how much yellow (specific color) there is?
        // Area of the largest contour (would require two calls to findContours() )?
        // How high the camera/claw is

        //TODO: play around with the kernal shape (ellipse, rect, etc),
        // try and cater size towards a vertical sample
        Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(kernalSize,kernalSize));
        Imgproc.erode(yellowRange, yellowRange, morphKernel);

        //Imgproc.findContours(redRange, redContours, new Mat(), Imgproc.CHAIN_APPROX_SIMPLE, Imgproc.RETR_TREE);
        //Imgproc.findContours(blueRange, blueContours, new Mat(), Imgproc.CHAIN_APPROX_SIMPLE, Imgproc.RETR_TREE);
        Imgproc.findContours(yellowRange, yellowContours, new Mat(), Imgproc.CHAIN_APPROX_SIMPLE, Imgproc.RETR_TREE);

        //Imgproc.drawContours(frame, redContours, -1, new Scalar(200,100,50), 3);
        //Imgproc.drawContours(frame, yellowContours, -1, new Scalar(50,100,200), 2);
        //Imgproc.drawContours(frame, blueContours, -1, new Scalar(100,200,100), 3);


        for (MatOfPoint contour : yellowContours){

            if (Imgproc.contourArea(contour) > minContourArea) {
                Rect box = Imgproc.boundingRect(contour);
                //if (Math.abs(( (double) box.height / box.width) - 1) < 0.5 && box.area() > minContourArea){
                yellowBoundingBoxes.add(box);
                //}


                RotatedRect rotBox = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                //if (rotBox.)
                yellowRotBoundingBoxes.add(rotBox);

                Point[] vertices = new Point[4];
                rotBox.points(vertices);

                for (int i = 0; i < 4; i++) {
                    Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(255, 255, 30), 1);
                }
            }
        }

        contourNum = yellowBoundingBoxes.size();

        for (Rect box : redBoundingBoxes){
            //Imgproc.rectangle(frame, box, new Scalar(200,150,0), 2);
        }

        for (Rect box : blueBoundingBoxes){
            //Imgproc.rectangle(frame, box, new Scalar(0,150,200), 2);
        }


        for (Rect box : yellowBoundingBoxes){
            //Imgproc.rectangle(frame, box, new Scalar(100,200,0), 2);
        }

        double minHeadingDist = 10000;
        bestBox = yellowRotBoundingBoxes.get(0);

        //TODO: fix the bounding box orientation
        //try drawing different rotated rects on the frame to better understand them

        for (int i = 0; i < yellowRotBoundingBoxes.size(); i++) {

            RotatedRect box = yellowRotBoundingBoxes.get(i);

            Imgproc.putText(frame, decForm1.format(Math.toDegrees(getAbsoluteRectAngle(box))), box.center, Imgproc.FONT_HERSHEY_PLAIN,0.7, new Scalar(180,30,30));
            Imgproc.circle(frame, box.center, 3, new Scalar(0,255,30));

            //TODO: Use homography to find (x,y) coordinate of each sample
            // use the field manhattan distance instead of camera coords

            //Find the manhattan distance to the box from the camera (increases the y distance to further penalize samples that we strafe to
            double manhattanDistance = xOffsetCoefficient * Math.abs(box.center.x - clawPosition.x) + Math.abs(box.center.y - clawPosition.y);
            double rectAngle = getAbsoluteRectAngle(box);
            //Find the offset of the box angle from 90 degrees
            double angleOffset = Math.toDegrees(Math.abs(rectAngle - (Math.PI/2)));

            double score = manhattanDistance + angleScoreCoefficient * angleOffset;

            double servoAngle = rectAngle / Math.PI;

            Imgproc.putText(frame, decForm1.format(score), new Point(box.center.x - 35, box.center.y), Imgproc.FONT_HERSHEY_PLAIN,0.7, new Scalar(30,30,150));
            Imgproc.putText(frame, decForm2.format(servoAngle), new Point(box.center.x - 15, box.center.y + 15), Imgproc.FONT_HERSHEY_PLAIN,0.7, new Scalar(30,150,30));

            sampleScores.add(score);

        }

        if (yellowRotBoundingBoxes.size() == 0){
            yellowRotBoundingBoxes.add(new RotatedRect());
            sampleScores.add(0.0);
        }

        double minScore = sampleScores.get(0);
        int bestSampleIndex = 0;

        for (int i = 0; i < sampleScores.size(); i++) {
            if (sampleScores.get(i) < minScore) {
                minScore = sampleScores.get(i);
                bestSampleIndex = i;
            }
        }

        bestBox = yellowRotBoundingBoxes.get(bestSampleIndex);

        Point[] verts = new Point[4];
        bestBox.points(verts);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(frame, verts[i], verts[(i + 1) % 4], new Scalar(230, 100, 30), 2);
        }

        contourX = bestBox.center.x;
        contourY = bestBox.center.y;
        //contourHead = minHeading;


//        if (boundingBoxes.size() > 0) {
//            meanContourValue = Core.mean(colorConvert.submat(boundingBoxes.get(0)));
//            Rect detection = boundingBoxes.get(0);
//            contourX = detection.x + (detection.width / 2.0);
//            contourY = detection.y + (detection.height / 2.0);
//            Imgproc.circle(frame, new Point(detection.x, detection.y), 2, new Scalar(255, 0, 0));
//            Imgproc.circle(frame, new Point(contourX, contourY), 4, new Scalar(255, 130, 0), 4);
//
//        }

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        //Imgproc.drawMarker(frame, contours.get(0).toList().get(0), new Scalar(255,40,40), Imgproc.MARKER_SQUARE, 10);


        return frame;
    }


    public void init(int width, int height, CameraCalibration calibration) {

    }


    public Object processFrame(Mat frame, long captureTimeNanos) {
        this.processFrame(frame);
        return null;
    }

    public Bitmap getLastFrame(){
        return lastFrame.get();
    }

    public int getYellowBoxes(){
        return yellowBoundingBoxes.size();
    }
    public int getRedBoxes(){
        return redBoundingBoxes.size();
    }
    public int getBlueBoxes(){
        return blueBoundingBoxes.size();
    }

    public double getAbsoluteRectAngle(RotatedRect rect) {
        Point[] vertices = new Point[4];
        rect.points(vertices);

        double dist1 = p2pDistance(vertices[0], vertices[1]);
        double dist2 = p2pDistance(vertices[1], vertices[2]);

        if (dist1 < dist2) {
            return Math.atan2(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y);
        } else {
            return Math.atan2(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y);
        }

    }

    public double p2pDistance(Point p1, Point p2) {
        return Math.hypot(p2.x - p1.x, p2.y - p1.y);
    }

}
