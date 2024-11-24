package org.firstinspires.ftc.teamcode.PathingSystems.PurePursuit;

import java.util.ArrayList;

public interface HermiteCurve {

    /**
     * Generates the Hermite Curve's general points at a certain speed.
     * @param speed Speed at which to generate points
     * @return The general list of points on the curve
     */
    public ArrayList<NavPoint> generateCurvePoints(double speed);

    /**
     * @return The general list of points on the curve from the last generation
     */
    public ArrayList<NavPoint> getCurvePoints();

    public int getSize();

}
