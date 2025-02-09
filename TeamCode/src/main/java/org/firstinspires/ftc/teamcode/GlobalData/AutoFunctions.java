package org.firstinspires.ftc.teamcode.GlobalData;

import com.pedropathing.pathgen.Point;

public class AutoFunctions {

    public static Point addPoints(Point p1, Point p2) {
        return new Point(p1.getX() + p2.getX(), p1.getY() + p2.getY());
    }

    public static double getDistance(Point p1, Point p2) {
        return Math.hypot(p2.getX() - p1.getX(),p2.getY() - p1.getY());
    }
}
