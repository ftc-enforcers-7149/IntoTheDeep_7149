package org.firstinspires.ftc.teamcode.Hardware.CoreMotion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.CoreMotion.ClosedLoop.ClosedLoopController;
import org.firstinspires.ftc.teamcode.Hardware.CoreMotion.EncoderResets.EncoderReset;

public class CoreController {

    private ClosedLoopController closedLoopController;
    private EncoderReset reset;
    private DcMotorEx motor;

    public CoreController(DcMotorEx motor, ClosedLoopController controller, EncoderReset resetter) {
        closedLoopController = controller;
        this.motor = motor;
        reset = resetter;
    }

    public void setTarget(double target) {

        if (reset != null) {
            if (reset.attemptReset(motor)) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        double power = closedLoopController.getPower(motor.getCurrentPosition(), target);
        motor.setPower(power);

    }

}
