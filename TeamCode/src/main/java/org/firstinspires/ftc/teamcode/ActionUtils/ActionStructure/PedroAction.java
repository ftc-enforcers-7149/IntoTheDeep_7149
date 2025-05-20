package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;


import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class PedroAction extends EventAction{

    private Follower follower;
    private PathChain pathChain;
    private boolean holdEnd;

    private boolean startedPath;

    public PedroAction(Follower follower, PathChain path, boolean holdEnd) {
        this.follower = follower;
        pathChain = path;
        this.holdEnd = holdEnd;

        startedPath = false;
    }


    @Override
    public boolean run(CombinedTelemetry t) {

        t.registerAction("Pedro Action", "t=" + follower.getCurrentTValue());

        if (!startedPath) {
            follower.followPath(pathChain, holdEnd);
            startedPath = true;
        }

        return follower.isBusy();
    }

    @Override
    public void init() {
        startedPath = false;
    }

    @Override
    public void stop(boolean interrupted) {
        startedPath = false;
        follower.breakFollowing();
    }
}
