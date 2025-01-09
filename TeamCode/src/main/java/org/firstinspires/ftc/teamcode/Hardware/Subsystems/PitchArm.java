package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

@Config
public class PitchArm implements PeriodicAction {

    public DcMotorEx pitchMotor;

    public static double kp = 0.012, ki = 0, kd = 0.0003;

    public static double ffCoefficient = 0;

    private PIDFController pitchController;

    public int initialPos;
    public int target;

    private boolean motorInterrupted;

    public PitchArm(HardwareMap hmap, String name) {
        pitchMotor = hmap.get(DcMotorEx.class, name);
        pitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pitchController = new PIDFController(kp, ki, kd, 0);

        initialPos = pitchMotor.getCurrentPosition();
        target = 0;

        motorInterrupted = false;
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        pitchController = new PIDFController(p, i, d, f);
    }

    public EventAction getPitchingAction(int targ) {
        return new PitchingAction(targ);
    }

    @Override
    public void periodic() {

        //pitchController.setF(Math.cos((pitchMotor.getCurrentPosition() - initialPos)/145.1 * 28/4 * 2 * Math.PI) * ffCoefficient);

        //always have PIDF running in background, unless motor is interrupted
        if (!motorInterrupted) {
            double pitchPow = pitchController.calculate(pitchMotor.getCurrentPosition(), target);
            pitchMotor.setPower(pitchPow);
        } else {
            pitchMotor.setPower(0);
        }

    }

    public class PitchingAction extends EventAction {

        public PitchingAction(int targ) {
            actionTarget = targ;
        }

        private int actionTarget;

        @Override
        public boolean run(CombinedTelemetry t) {
            isRunning = true;

            t.getTelemetry().addData("Pitching", actionTarget + " " + pitchMotor.getCurrentPosition());

            target = actionTarget;
            //if it is not at its target position, then the action is still running
            return Math.abs(pitchMotor.getCurrentPosition() - target) > 10;
        }

        @Override
        public void init() {
            pitchController.reset();
            motorInterrupted = false;
        }

        @Override
        public void stop(boolean interrupted) {
            isRunning = false;
            motorInterrupted = interrupted;
        }
    }
}
