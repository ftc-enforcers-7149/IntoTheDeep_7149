package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;


/**
 * Polygon abstract superclass made using Pedro Pathing geometry classes
 */
public abstract class Shape {

    abstract public boolean withinBounds(Pose position);

    public boolean withinBounds(Point position) {
        return withinBounds(new Pose(position.getX(), position.getY()));
    }

    public boolean withinBounds(Pose2d position) {
        return withinBounds(new Pose(position.position.x, position.position.y));
    }

    abstract public double getArea();
}
