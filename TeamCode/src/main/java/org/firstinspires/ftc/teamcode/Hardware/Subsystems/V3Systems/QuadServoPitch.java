package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

@Config
public class QuadServoPitch implements PeriodicAction {

    //as viewed from the front of the robot
    private CRServo leftServoBottom, leftServoTop, rightServoBottom, rightServoTop;
    private AxonAbsoluteEncoder pitchEncoder;
    private PIDFController controller;

    private OuttakeExtendo extendoForPIDF;

    private double pitchPos, target, pitchAngle;
    private final double POS_PITCH_CONVERSION = 1/3.0;

    public static double kpMin = 0, kdMin = 0, kpMax = 0, kdMax = 0;
    public static double ffMin = 0, ffMax = 0;

    public QuadServoPitch(OpMode opMode) {
        leftServoBottom = opMode.hardwareMap.get(CRServo.class, "leftBottom");
        leftServoTop = opMode.hardwareMap.get(CRServo.class, "leftTop");
        rightServoBottom = opMode.hardwareMap.get(CRServo.class, "rightBottom");
        rightServoTop = opMode.hardwareMap.get(CRServo.class, "rightTop");

        controller = new PIDFController(kpMin,0,kdMin,0);

        pitchEncoder = new AxonAbsoluteEncoder(opMode.hardwareMap, "pitchInput");
        extendoForPIDF = null;

        pitchPos = pitchEncoder.getAbsolutePosition();
    }

    public void attachExtendo(OuttakeExtendo ext) {
        extendoForPIDF = ext;
    }

    public void setPower(double power) {
        leftServoBottom.setPower(power);
        leftServoTop.setPower(power);
        rightServoBottom.setPower(power);
        rightServoTop.setPower(power);
    }

    /**
     * @return The position of the pitch from wraparound encoder
     */
    public double getPosition() {
        return pitchPos;
    }

    public double getPitchAngle() {
        return pitchAngle;
    }

    public void updateEncoder() {
        pitchEncoder.periodic();
    }

    @Override
    public void periodic() {

        double currentFF = 0;
        //updates PIDF values based on the position of the extendo
        //interpolates pidf values linearly since torque changes linearly (is proportional to length of extendo)
        if (extendoForPIDF == null) {
            controller.setPIDF(kpMin, 0, kdMin, 0);
            currentFF = ffMin;
        } else {
            double extPosPercent = extendoForPIDF.getExtensionPos() / OuttakeExtendo.MAX_EXTENSION;
            controller.setPIDF(kpMin + (kpMax - kpMin) * extPosPercent,
                    0, kdMin + (kdMax - kdMin) * extPosPercent, 0);
            currentFF = ffMin + (ffMax - ffMin) * extPosPercent;
        }

        //update the encoder inside this periodic call
        pitchEncoder.periodic();

        //get position from encoder and convert to an angle measure
        pitchPos = pitchEncoder.getAbsolutePosition();
        pitchAngle = pitchPos * POS_PITCH_CONVERSION;

        //adds angle based feedforward
        double pitchPow = controller.calculate(pitchPos, target) + (Math.cos(Math.toRadians(pitchAngle)) * currentFF);

        setPower(pitchPow);
    }

    /**
     * For testing purposes only. To properly set the target, use a PitchingAction
     * @param t
     */
    public void setTarget(int t) {
        target = t;
    }

    public EventAction getPitchingAction(int targ) {
        return new PitchingAction(targ);
    }

    public class PitchingAction extends EventAction {

        public PitchingAction(int targ) {
            actionTarget = targ;
        }

        private int actionTarget;

        @Override
        public boolean run(CombinedTelemetry t) {
            isRunning = true;

            double pos = getPosition();

            t.getTelemetry().addData("Pitching", "Target: " + actionTarget + " Pos: " + pos);

            target = actionTarget;
            //if it is not at its target position, then the action is still running
            return Math.abs(pos - target) > 10;
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
