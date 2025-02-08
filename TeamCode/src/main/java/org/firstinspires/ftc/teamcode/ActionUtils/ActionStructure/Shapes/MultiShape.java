package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.Shapes;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

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
