package org.firstinspires.ftc.teamcode.PurePursuit;

public class PurePursuitReturn{

    private NavPoint goalPt;

    private int lastFoundIndex;

    public PurePursuitReturn(NavPoint pt, int index) {
        goalPt = pt;
        lastFoundIndex = index;
    }

    public NavPoint getGoalPoint(){
        return goalPt;
    }

    public int getLastFoundIndex(){
        return lastFoundIndex;
    }

}
