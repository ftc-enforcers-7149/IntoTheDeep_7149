package org.firstinspires.ftc.teamcode.Hardware.Chassis;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PeriodicAction;

public class PeriodicFollower implements PeriodicAction {

    private Follower follower;

    public PeriodicFollower(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void periodic() {
        follower.update();
    }
}
