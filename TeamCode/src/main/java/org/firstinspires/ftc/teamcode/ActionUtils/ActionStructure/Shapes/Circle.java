package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;

public class Circle extends Shape{

    private Point center;

    private double radius;

    public Circle(double radius, Point center) {
        this.radius = radius;
        this.center = center;
    }

    public Circle(double radius) {
        this(radius, new Point(0,0));
    }

    @Override
    public boolean withinBounds(Pose position) {
        return MathFunctions.distance(position, center) <= radius;
    }

    @Override
    public double getArea() {
        return Math.PI * Math.pow(radius, 2);
    }
}
