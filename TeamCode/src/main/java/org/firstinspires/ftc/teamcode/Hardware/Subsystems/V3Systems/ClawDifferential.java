package org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;

public class ClawDifferential {

    //as viewed with front facing away
    private ServoImplEx leftServo, rightServo;
    private double leftPos, rightPos;

    private double pitchAngle, rotAngle; //pitch(0, 170)  rot(0,180)

    /**
     * Initializes a ClawDifferential object with initial pitch angle of -80 degrees
     * and initial rotation of 180 degrees.
     * @param opmode Current opMode creating this object
     */
    public ClawDifferential(OpMode opmode) {

        leftServo = opmode.hardwareMap.get(ServoImplEx.class, "leftDiffy");
        rightServo = opmode.hardwareMap.get(ServoImplEx.class, "rightDiffy");
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        pitchAngle = 0;
        rotAngle = 180;

    }

    public void initPositions() {
        leftPos =  180/355.0;
        rightPos = 0;

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);
    }

    /**
     * Sets the pitch position of the differential claw
     * @param pitch in degrees
     */
    public void setPitchAngle(double pitch) {

        pitch = Math.min(Math.max(pitch, -80), 90);  //make sure minimum position is -80
                                                    // so servos have travel range of 350 degrees
        pitch += 80;  //offset pitch such that the minimum position is 0

        leftPos = leftPos + (pitch - pitchAngle)/355;
        rightPos = rightPos + (pitch - pitchAngle)/355;

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);

        pitchAngle = pitch;

    }

    /**
     * Sets the rotation position of the differential claw
     * @param rot in degrees
     */
    public void setRotationAngle(double rot) {

        rot = Math.max(Math.min(rot, 180), 0);

        leftPos = leftPos + (rot - rotAngle)/355;
        rightPos = rightPos - (rot - rotAngle)/355;

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);

        rotAngle = rot;

    }

    public void setPitchRotAngles(double pitch, double rot) {

        rot = Math.max(Math.min(rot, 180), 0);
        pitch = Math.min(Math.max(pitch, -80), 90);  //make sure minimum position is -80
        // so servos have travel range of 350 degrees
        pitch += 80;  //offset pitch such that the minimum position is 0

        leftPos = leftPos + (rot - rotAngle)/355 + (pitch - pitchAngle)/355;
        rightPos = rightPos - (rot - rotAngle)/355 + (pitch - pitchAngle)/355;

        rotAngle = rot;
        pitchAngle = pitch;

    }

    public double getPitchAngle() {
        return pitchAngle;
    }

    public double getRotationAngle() {
        return rotAngle;
    }

    public double getLeftPos() {
        return leftPos;
    }

    public double getRightPos() {
        return rightPos;
    }

    public EventAction getGamepadRotationAction(Gamepad g) {
        return new GamepadRotation(g);
    }

    class GamepadRotation extends EventAction{

        private Gamepad gamepad;
        private double pos;
        private final double speed = 5;

        public GamepadRotation(Gamepad gpad) {
            gamepad = gpad;
            pos = rotAngle;
        }

        @Override
        public boolean run(CombinedTelemetry t) {

            pos += Range.clip(pos + (speed * gamepad.left_stick_x), 0, 180);

            setRotationAngle(pos);

            return false;
        }

        @Override
        public void init() {
            pos = rotAngle;
        }

        @Override
        public void stop(boolean interrupted) {

        }
    }

}
