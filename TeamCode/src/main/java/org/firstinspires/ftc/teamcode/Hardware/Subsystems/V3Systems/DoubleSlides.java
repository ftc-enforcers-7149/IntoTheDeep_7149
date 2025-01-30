package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

public class DoubleSlides implements PeriodicAction {

    //as viewed from the front of the robot
    private DcMotorEx leftMotor, rightMotor;
    private PIDFController controller;
    private int target;

    public DoubleSlides(OpMode opMode) {
        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "leftSlides");
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "rightSlides");

        controller = new PIDFController(0,0,0,0);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int getCurrentPosition() {
        int leftPos = leftMotor.getCurrentPosition();
        int rightPos = rightMotor.getCurrentPosition();

        //if an encoder gets unplugged, use the other
        if (leftPos == 0 && rightPos != 0) {
            return rightPos;
        } else if (rightPos == 0 && leftPos != 0) {
            return leftPos;
        } else {
            return (leftPos + rightPos) / 2;
        }
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    @Override
    public void periodic() {

        double slidePow = controller.calculate(getCurrentPosition(), target);
        setPower(slidePow);

    }

    /**
     * For testing purposes only. To properly set the target, use an ExtensionAction
     * @param t
     */
    public void setTarget(int t) {
        target = t;
    }

    public EventAction getExtensionAction(int targ) {
        return new ExtensionAction(targ);
    }

    public class ExtensionAction extends EventAction{

        public ExtensionAction(int targ) {
            actionTarget = targ;
        }

        private int actionTarget;

        @Override
        public boolean run(CombinedTelemetry t) {
            isRunning = true;

            double pos = getCurrentPosition();

            t.registerAction("Extension", "Target: " + actionTarget + " Pos: " + pos);

            target = actionTarget;

            //if it is not at its target position, then the action is still running
            return Math.abs(pos - target) > 15;
        }

        @Override
        public void init() {
            controller.reset();
        }

        @Override
        public void stop(boolean interrupted) {
            isRunning = false;
        }
    }


}
