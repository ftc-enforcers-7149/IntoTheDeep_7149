package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.OuttakeSlides;

public class TwoMotorSlides implements PeriodicAction {

    private DcMotorEx leftMotor, rightMotor;
    private PIDFController controller;

    private int target;
    private boolean motorInterrupted;

    public TwoMotorSlides(OpMode opMode) {
        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "frontSlidesLeft");
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "frontSlidesRight");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDFController(0.0042, 0, 0.0003, 0.00009);

        target = 0;
        motorInterrupted = false;
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public int getPosition() {
        return rightMotor.getCurrentPosition();
    }

    public double getVelocity() {
        return rightMotor.getVelocity();
    }

    public void setMode(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    @Override
    public void periodic() {

        if (!motorInterrupted) {
            double slidePow = controller.calculate(this.getPosition(), target);
            this.setPower(slidePow);
        } else {
            this.setPower(0);
        }
    }


    public EventAction getExtensionAction(int targ) {
        return new ExtensionAction(targ);
    }

    public class ExtensionAction extends EventAction {

        public ExtensionAction(int targ) {
            actionTarget = targ;
        }

        private int actionTarget;

        @Override
        public boolean run(CombinedTelemetry t) {
            isRunning = true;

            t.registerAction("Extension", "Target: " + actionTarget + " Pos: " + rightMotor.getCurrentPosition());

            target = actionTarget;

            //if it is not at its target position, then the action is still running
            return Math.abs(rightMotor.getCurrentPosition() - target) > 30;
        }

        @Override
        public void init() {
            controller.reset();
            motorInterrupted = false;
        }

        @Override
        public void stop(boolean interrupted) {
            isRunning = false;
            motorInterrupted = interrupted;
        }
    }
}
