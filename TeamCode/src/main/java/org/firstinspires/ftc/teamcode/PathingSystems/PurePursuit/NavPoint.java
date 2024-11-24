package org.firstinspires.ftc.teamcode.PathingSystems.PurePursuit;

import com.acmerobotics.roadrunner.Pose2d;

public class NavPoint {

    private double x;
    private double y;
    private double theta;

    /**
     *
     * @param x1 in inches
     * @param y1 in inches
     * @param heading in radians
     */
    public NavPoint(double x1, double y1, double heading){
        x = x1;
        y = y1;
        theta = heading;
    }

    public NavPoint(double x1, double y1) {
        this(x1, y1, 0);
    }

    public NavPoint setHeading(double t) {
        theta = t;
        return this;
    }

    public double getX(){
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading(){
        return theta;
    }

    public Pose2d toPose() {
        return new Pose2d(x, y, theta);
    }

}
