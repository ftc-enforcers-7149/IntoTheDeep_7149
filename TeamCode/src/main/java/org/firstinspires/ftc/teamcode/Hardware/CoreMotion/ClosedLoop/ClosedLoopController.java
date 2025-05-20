package org.firstinspires.ftc.teamcode.Hardware.CoreMotion.ClosedLoop;

public interface ClosedLoopController {

    public double getPower(double pos, double target);

    public void setCoefficients(double ... coefficients);

}
