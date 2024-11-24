package org.firstinspires.ftc.teamcode.PathingSystems.PurePursuit;

public class PIDF {

    private double kP, kI, kD, FF;

    private double totalError, lastError;

    private double target;


    public PIDF(double p, double i, double d, double f){
        setCoefficients(p, i, d, f);
        reset();
        target = 0;
    }

    public void setCoefficients(double p, double i, double d, double f){
        kP = p;
        kI = i;
        kD = d;
        FF = f;
    }

    public void reset(){
        totalError = 0;
        lastError = 0;
    }

    public void setTarget(double t){
        target = t;
    }

    public double calculate(double state, double loopTime){

        double error = target - state;

        double derivative = (lastError - error) / loopTime;

        totalError += error * loopTime;

        lastError = error;

        return error * kP + totalError * kI + derivative * kD + FF;
    }



}
