package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes;

import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Point;

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
