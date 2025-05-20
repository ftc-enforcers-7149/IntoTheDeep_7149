package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;

public class MultiShape extends Shape{

    private Shape[] shapes;

    public MultiShape(Shape...shapes) {
        this.shapes = shapes;
    }

    @Override
    public boolean withinBounds(Pose position) {

        for (Shape shape : shapes) {
            if (shape.withinBounds(position)) {
                return true;
            }
        }

        return false;
    }

    @Override
    public double getArea() {
        double totalArea = 0;

        for (Shape shape : shapes) {
            totalArea += shape.getArea();
        }

        return totalArea;
    }
}
