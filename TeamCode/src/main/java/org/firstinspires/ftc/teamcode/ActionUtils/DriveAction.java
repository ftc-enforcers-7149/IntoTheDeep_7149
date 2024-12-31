package org.firstinspires.ftc.teamcode.ActionUtils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;

public class DriveAction extends EventAction {

    private MecanumPowerDrive drive;
    private Gamepad gamepad;

    private boolean fieldCentricMode;
    private double deadzoneCoefficient;

    private double lastHeadingTrack = 0;


    public DriveAction(MecanumPowerDrive dr, Gamepad gpad, boolean fieldCentric, double deadzone) {
        drive = dr;
        gamepad = gpad;

        fieldCentricMode = fieldCentric;
        deadzoneCoefficient = deadzone;

        lastHeadingTrack = dr.getLocalizerPose().heading.toDouble();
    }

    @Override
    public boolean run(CombinedTelemetry t) {


        //set different driving modes
        if (fieldCentricMode) {

            //if the robot isn't being manually turned and the robot is no longer turning, auto-lock the heading
            if (Math.abs(gamepad.right_stick_x) < 0.01 && Math.abs(drive.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) < 0.05) {

                double currentHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                //lock on to the last heading the robot was at prior to auto-lock

                drive.setFieldCentricPower(gamepad.left_stick_x * deadzoneCoefficient,
                        -1 * gamepad.left_stick_y * deadzoneCoefficient,
                        drive.headingDriftPID.calculate(currentHeading, lastHeadingTrack));

            } else {

                drive.setFieldCentricPower(gamepad.left_stick_x * deadzoneCoefficient,
                        -1 * gamepad.left_stick_y * deadzoneCoefficient,
                        gamepad.right_stick_x * deadzoneCoefficient);

                //update the last heading to be tracked once auto-lock is engaged
                lastHeadingTrack = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
        } else {
            drive.setRobotCentricPower(gamepad.left_stick_x * deadzoneCoefficient,
                                       -1 * gamepad.left_stick_y * deadzoneCoefficient,
                                       gamepad.right_stick_x * deadzoneCoefficient);
        }

        //always continue running the drive action
        return true;
    }

    @Override
    public void init() {

    }

    @Override
    public void stop(boolean interrupted) {
        drive.setRobotCentricPower(0,0,0);
    }
}
