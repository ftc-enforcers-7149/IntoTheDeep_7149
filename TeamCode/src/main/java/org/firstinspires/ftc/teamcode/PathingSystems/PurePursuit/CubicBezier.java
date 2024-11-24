package org.firstinspires.ftc.teamcode.PathingSystems.PurePursuit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CubicBezier implements HermiteCurve{

    private ArrayList<NavPoint> controlPoints;
    private ArrayList<NavPoint> curvePoints;

    public CubicBezier(double speed, List<NavPoint> ctrlPoints) {
        controlPoints = new ArrayList<NavPoint>((List<NavPoint>) ctrlPoints);

        if (controlPoints.size() < 4) {
            throw new RuntimeException("Insufficient Control Points");
        }

        generateCurvePoints(speed);
    }

    public CubicBezier(double speed, NavPoint...ctrlPoints) {
        this(speed, Arrays.asList(ctrlPoints));
    }

    @Override
    public ArrayList<NavPoint> generateCurvePoints(double speed) {
        double t = 0;
        curvePoints.clear();

        while (t <= 1) {

            double p0x = Math.pow((1-t), 3) * controlPoints.get(0).getX();
            double p0y = Math.pow((1-t), 3) * controlPoints.get(0).getY();

            double p1x = 3 * Math.pow((1-t), 2) * t * controlPoints.get(1).getX();
            double p1y = 3 * Math.pow((1-t), 2) * t * controlPoints.get(1).getX();

            double p2x = 3 * (1-t) * Math.pow(t,2) * controlPoints.get(2).getX();
            double p2y = 3 * (1-t) * Math.pow(t,2) * controlPoints.get(2).getX();

            double p3x = Math.pow(t,3) * controlPoints.get(3).getX();
            double p3y = Math.pow(t,3) * controlPoints.get(3).getX();

            curvePoints.add(new NavPoint(p0x + p1x + p2x + p3x, p0y + p1y + p2y + p3y));

            t += speed;
        }
        return curvePoints;
    }

    @Override
    public ArrayList<NavPoint> getCurvePoints() {
        return curvePoints;
    }

    @Override
    public int getSize() {
        return curvePoints.size();
    }
}
