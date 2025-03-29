package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

@Config
public class QuadServoPitch implements PeriodicAction {

    //as viewed from the front of the robot
    private CRServo leftServoBottom, leftServoTop, rightServoBottom, rightServoTop;
    private AxonPowerEncoder pitchEncoder;
    private PIDFController controller;

    private OuttakeExtendo extendoForPIDF;
    private Telemetry tele;

    //private DcMotorEx encoderMotor;   //TODO: If you want to use rev through bore encoder, uncomment all the dc motor lines in this file, and check the other TODOs

    private double pitchPos, target, pitchAngle;
    private double lastPower;
    private final double POS_PITCH_CONVERSION = 1/3.0;

    public static double kpMin = 0.006, kdMin = 0.0001, ffMin = -0.07, kpMax = 0, kdMax = 0;
    public static double ffMax = 0;

    public QuadServoPitch(OpMode opMode) {
        leftServoBottom = opMode.hardwareMap.get(CRServo.class, "left1");
        leftServoTop = opMode.hardwareMap.get(CRServo.class, "left2");
        rightServoBottom = opMode.hardwareMap.get(CRServo.class, "right1");
        rightServoTop = opMode.hardwareMap.get(CRServo.class, "right2");

        controller = new PIDFController(kpMin,0,kdMin,0);

        pitchEncoder = new AxonPowerEncoder(opMode, "pitchInput");
        tele = opMode.telemetry;
        extendoForPIDF = null;

        //TODO
//        encoderMotor = opMode.hardwareMap.get(DcMotorEx.class, "");
//        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pitchPos = pitchEncoder.getAbsolutePosition();
        lastPower = 777;
    }

    public void attachExtendo(OuttakeExtendo ext) {
        extendoForPIDF = ext;
    }

    public void setPower(double power) {

        if (power != lastPower) {
            leftServoBottom.setPower(power);
            leftServoTop.setPower(power);
            rightServoBottom.setPower(power);
            rightServoTop.setPower(power);
        }

        lastPower = power;
    }

    /**
     * @return The position of the pitch from wraparound encoder
     */
    public double getPosition() {
        return pitchEncoder.getAbsolutePosition();
        //return encoderMotor.getCurrentPosition();  //TODO and comment out the other return statement here
    }

    public double getRawPosition() { return pitchEncoder.getCurrentPosition(); }

    public double getPitchAngle() {
        return pitchAngle;
    }

    public void updateEncoder() {
        pitchEncoder.update(lastPower);
    }

    @Override
    public void periodic() {

        double currentFF = 0;
        //updates PIDF values based on the position of the extendo
        //interpolates pidf values linearly since torque changes linearly (is proportional to length of extendo)
        //if (extendoForPIDF == null) {
        controller.setPIDF(kpMin, 0, kdMin, 0);
        currentFF = ffMin;
//        } else {
//            double extPosPercent = extendoForPIDF.getExtensionPos() / OuttakeExtendo.MAX_EXTENSION;
//            controller.setPIDF(kpMin + (kpMax - kpMin) * extPosPercent,
//                    0, kdMin + (kdMax - kdMin) * extPosPercent, 0);
//            currentFF = ffMin + (ffMax - ffMin) * extPosPercent;
//        }

        //update the encoder inside this periodic call
        updateEncoder();

        //get position from encoder and convert to an angle measure
        pitchPos = pitchEncoder.getAbsolutePosition();
        pitchAngle = Range.scale(pitchPos, 350, 50, 0, 90);

        tele.addData("Pitch angle", pitchAngle);

        //adds angle based feedforward

        //TODO: if you want to flip the pitch direction, change the -1 below to +1, and also look below
        double pitchPow = -1 * controller.calculate(pitchPos, target) + (Math.sin(Math.toRadians(pitchAngle)) * currentFF);


        //TODO: to flip direction, change the -0.25s to +0.25
        //TODO: if using the motor, change target to the position at which the pitch arm performs a pass through
        if (target > 360 && pitchPow < -0.25) {
            pitchPow = -0.25;
        }

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
