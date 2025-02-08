package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class Rectangle extends Shape {

    private double height, width;

    private Point[] vertices;

    public Rectangle(Point topLeft, Point topRight, Point bottomLeft, Point bottomRight) {
        this.vertices = new Point[] {topLeft, topRight, bottomLeft, bottomRight};

        height = MathFunctions.distance(vertices[1], vertices[2]);
        width = MathFunctions.distance(vertices[0], vertices[1]);
    }

    public Rectangle(Point topLeft, Point bottomRight) {
        height = Math.abs(topLeft.getY() - bottomRight.getY());
        width = Math.abs(bottomRight.getX() - topLeft.getX());

        this.vertices = new Point[] {
                topLeft,
                MathFunctions.addPoints(topLeft, new Point(width, 0)),    //topRight
                MathFunctions.addPoints(topLeft, new Point(0, -height)),  //bottomLeft
                MathFunctions.addPoints(topLeft, new Point(width, -height))    //bottomRight
        };
    }

    public Rectangle(double height, double width, Point topLeft) {
        this.height = height;
        this.width = width;

        this.vertices = new Point[] {
                topLeft,
                MathFunctions.addPoints(topLeft, new Point(width, 0)),    //topRight
                MathFunctions.addPoints(topLeft, new Point(0, -height)),  //bottomLeft
                MathFunctions.addPoints(topLeft, new Point(width, -height))    //bottomRight
        };

    }

    @Override
    public boolean withinBounds(Pose position) {
        double posX = position.getX();
        double posY = position.getY();

        return (posX > vertices[0].getX() && posX < vertices[1].getX())
                && (posY < vertices[0].getY() && posY > vertices[2].getY());
    }

    @Override
    public double getArea() {
        return width * height;
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    public Point[] getVertices() { return vertices;}
}
