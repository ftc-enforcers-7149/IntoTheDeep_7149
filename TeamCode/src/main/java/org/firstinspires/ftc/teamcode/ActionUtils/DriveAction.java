package org.firstinspires.ftc.teamcode.ActionUtils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;

public class DriveAction extends EventAction {

    private MecanumPowerDrive drive;
    private Gamepad gamepad;

    private boolean fieldCentricMode;
    private double deadzoneCoefficient;


    public DriveAction(MecanumPowerDrive dr, Gamepad gpad, boolean fieldCentric, double deadzone) {
        drive = dr;
        gamepad = gpad;

        fieldCentricMode = fieldCentric;
        deadzoneCoefficient = deadzone;
    }

    @Override
    public boolean run(CombinedTelemetry t) {


        //set different driving modes
        if (fieldCentricMode) {
            drive.setFieldCentricPower(gamepad.left_stick_x * deadzoneCoefficient,
                                       -1 * gamepad.left_stick_y * deadzoneCoefficient,
                                       gamepad.right_stick_x * deadzoneCoefficient);
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
