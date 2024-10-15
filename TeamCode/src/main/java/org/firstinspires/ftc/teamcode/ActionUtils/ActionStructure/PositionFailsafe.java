package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis.MecanumPowerDrive;

public class PositionFailsafe implements FailsafeTrigger{

    private MecanumPowerDrive drive;
    private Pose2d target;

    private int offenseNum;
    private int maxOffenses;
    private double toleranceDist;
    private int timeOutMs;

    private boolean triggered;
    private boolean offenseTriggered, lastOffenseTriggered;

    private ElapsedTime timer;

    public PositionFailsafe(MecanumPowerDrive dr, Pose2d targ, double tolerance, int maxOffenseNum, int maxTimeMs) {
        drive = dr;
        target = targ;

        toleranceDist = tolerance;
        maxOffenses = maxOffenseNum;
        timeOutMs = maxTimeMs;

        timer = new ElapsedTime();

        offenseNum = 0;
        triggered = false;
        offenseTriggered = false;
        lastOffenseTriggered = false;
    }

    @Override
    public boolean triggered() {

        Pose2d pose = drive.pose;

        offenseTriggered = ( Math.hypot(pose.position.x - target.position.x, pose.position.y - target.position.y)
                > toleranceDist );

        //if this is the first time pose goes past tolerance, add an offense
        //and start timeOut timer
        //Doesn't continuously add offenses if it stays outside of tolerance
        if (offenseTriggered && !lastOffenseTriggered) {
            offenseNum += 1;
            timer.reset();
        }

        //if position stays outside of tolerance for longer than timeOut, then trigger failsafe
        if (offenseTriggered && lastOffenseTriggered && timer.milliseconds() > timeOutMs) {
            triggered = true;
        }

        lastOffenseTriggered = offenseTriggered;

        if (offenseNum >= maxOffenses) {
            triggered = true;
        }

        return triggered;

    }

}
