package org.firstinspires.ftc.teamcode.VisionProcessors;

//import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class ProcessorBase extends OpenCvPipeline /*implements VisionProcessor, CameraStreamSource*/ {

    public static Scalar lowerRed = new Scalar(215,50,50);
    public static Scalar upperRed = new Scalar(255,200,225);
    public static Scalar lowerBlue = new Scalar(145,50,50);
    public static Scalar upperBlue = new Scalar(175,200,225);
    public static Scalar lowerYellow = new Scalar(25,110,200); //(10,95,95);
    public static Scalar upperYellow = new Scalar(45,255,255);//(45,255,255);



    public static double getAbsoluteRectAngle(RotatedRect rect) {
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

    public static double p2pDistance(Point p1, Point p2) {
        return Math.hypot(p2.x - p1.x, p2.y - p1.y);
    }

    private static Point multiplyPointByHomography(Point point, Mat homography) {
        MatOfPoint2f src = new MatOfPoint2f(point);
        MatOfPoint2f dst = new MatOfPoint2f();

        Core.perspectiveTransform(src, dst, homography);

        return dst.toList().get(0);
    }


}
