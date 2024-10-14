package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

public class OuttakeSlides implements PeriodicAction {

    public DcMotorEx slideMotor;

    public static double kp = 0.008, ki = 0, kd = 0.00026, ff=0.00012;

    private PIDFController slideController;

    public int initialPos;
    public int target;

    private boolean motorInterrupted;

    public OuttakeSlides(HardwareMap hmap, String name) {
        slideMotor = hmap.get(DcMotorEx.class, name);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideController = new PIDFController(kp, ki, kd, ff);

        initialPos = slideMotor.getCurrentPosition();
        target = 0;

        motorInterrupted = false;
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        slideController.setPIDF(p, i, d, f);
    }

    public EventAction getExtensionAction(int targ) {
        return new ExtensionAction(targ);
    }

    @Override
    public void periodic() {

        //always have PIDF running in background, unless motor is interrupted
        if (!motorInterrupted) {
            double slidePow = slideController.calculate(slideMotor.getCurrentPosition(), target);
            slideMotor.setPower(slidePow);
        } else {
            slideMotor.setPower(0);
        }

    }

    public class ExtensionAction implements EventAction{

        public ExtensionAction(int targ) {
            actionTarget = targ;
        }

        private int actionTarget;

        @Override
        public boolean run(CombinedTelemetry t) {

            t.getTelemetry().addData("Extension", actionTarget);

            target = actionTarget;
            //if it is not at its target position, then the action is still running
            return Math.abs(slideMotor.getCurrentPosition() - target) > 30;
        }

        @Override
        public void init() {
            slideController.reset();
            motorInterrupted = false;
        }

        @Override
        public void stop(boolean interrupted) {
            motorInterrupted = interrupted;
        }
    }
}
