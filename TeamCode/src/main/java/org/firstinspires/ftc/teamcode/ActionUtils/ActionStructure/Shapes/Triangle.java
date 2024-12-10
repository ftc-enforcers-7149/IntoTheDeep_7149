package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes;

import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Point;

public class Triangle extends Shape{

    private Point[] vertices;

    private double[] sideLengths;
    private double[] angles;

    public Triangle(Point left, Point center, Point right) {
        vertices = new Point[] {left, center, right};

        sideLengths = new double[] {
                MathFunctions.distance(left, center),
                MathFunctions.distance(center, right),
                MathFunctions.distance(right, left),
        };

        angles = new double[3];

        //c^2 = a^2 + b^2 - abcos(C)
        angles[0] = Math.acos(( Math.pow(sideLengths[1], 2) - (Math.pow(sideLengths[0], 2) + Math.pow(sideLengths[2],2)) ) /
                (-sideLengths[0] * sideLengths[2]));
        angles[1] = Math.acos(( Math.pow(sideLengths[2], 2) - (Math.pow(sideLengths[1], 2) + Math.pow(sideLengths[0],2)) ) /
                (-sideLengths[1] * sideLengths[0]));
        angles[2] = 180 - angles[0] - angles[1];
    }

    //Thanks to Kornel Kisielewicz and xaedes (Stack Overflow)
    @Override
    public boolean withinBounds(Pose position) {

        double d1, d2, d3;
        boolean has_neg, has_pos;

        Point pos = new Point(position.getX(), position.getY());

        d1 = sign(pos, vertices[0], vertices[1]);
        d2 = sign(pos, vertices[1], vertices[2]);
        d3 = sign(pos, vertices[2], vertices[0]);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    @Override
    public double getArea() {
        return 0.5 * sideLengths[0] * sideLengths[2] * Math.sin(angles[0]);
    }

    public double sign(Point p1, Point p2, Point p3)
    {
        return (p1.getX() - p3.getX()) * (p2.getY() - p3.getY()) - (p2.getX() - p3.getX()) * (p1.getY() - p3.getY());
    }

    public Point[] getVertices() {
        return vertices;
    }

    public double[] getSideLengths() {
        return sideLengths;
    }

    public double[] getAngles() {
        return angles;
    }
}
