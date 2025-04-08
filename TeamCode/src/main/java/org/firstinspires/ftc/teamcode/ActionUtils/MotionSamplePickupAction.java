package org.firstinspires.ftc.teamcode.ActionUtils;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.WaitAction;
import org.firstinspires.ftc.teamcode.GlobalData.HardwareConstants;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.LLSampleVision;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.util.Range;

public class MotionSamplePickupAction extends EventAction {

    private SequentialAction pickupAction, pickupCopy;
    private ParallelAction aboveAction, aboveCopy;
    private EventAction autoAction, findNewSample, totalAction;

    private LLSampleVision visionSystem;
    private Servo clawWrist;
    private Follower follower;

    private boolean positionReceived, findingNewSample;  //true once the robot has a sample it can go after

    //TODO:
    // Encode x, y, and rot values into one double for limelight results
    // Eg. x = 12.39, y = 3.98, rot = 152.49 --> 11239039815249.0
    // Start with a 1 each time to maintain starting 0
    // then x, y, and rot are encoded in the double

    //TODO:
    // Limelight should properly encode positions and add the 1 at the start
    // Limelight should also order sample info with closest sample first, then second closest, and on

    public MotionSamplePickupAction(LLSampleVision vision, Servo wrist, Follower follow, ParallelAction aboveSample, SequentialAction pickupSample) {
        pickupAction = pickupSample;
        aboveAction = aboveSample;

        pickupCopy = pickupAction.copy();
        aboveCopy = aboveAction.copy();

        visionSystem = vision;
        clawWrist = wrist;
        follower = follow;

        autoAction = pickupAction;
        totalAction = pickupAction;
        findNewSample = null;

        positionReceived = false;
        findingNewSample = false;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        //if a sample hasn't been chosen, find one
        if (!positionReceived) {

            double[] sampleInfo;

            if (!findingNewSample) {
                sampleInfo = visionSystem.getResultInfo();
            } else {
                sampleInfo = new double[]{0,0,0,0,0,0,0,0};
            }

            if (sampleInfo[0] == 1) {
                positionReceived = true;


                double heading = follower.getPose().getHeading();
                Point currentPos = new Point(follower.getPose().getX(), follower.getPose().getY());

                Point moveVectorX = new Point(sampleInfo[2], (heading - Math.PI/2), Point.POLAR);
                Point moveVectorY = new Point(-1 * (HardwareConstants.MAX_EXTENDO_LENGTH - sampleInfo[3]), (heading), Point.POLAR);
                Point moveVector = MathFunctions.addPoints(moveVectorX, moveVectorY);

                Path moveToPoint = new Path(new BezierLine(
                        currentPos,
                        MathFunctions.addPoints(currentPos, moveVector)
                ));
                moveToPoint.setConstantHeadingInterpolation(heading);
                moveToPoint.setZeroPowerAccelerationMultiplier(1);

                autoAction = new PedroAction(follower, new PathChain(moveToPoint), true);

                clawWrist.setPosition(1 - Range.scale(sampleInfo[1] / 180, 0, 1, HardwareConstants.WRIST_MIN, HardwareConstants.WRIST_MAX));
                totalAction = new SequentialAction(
                        new ParallelAction(autoAction, aboveAction),
                        pickupAction);

                totalAction.init();
                return totalAction.run(t);

            } else {

                findingNewSample = true;

                if (findNewSample == null) {

                    double heading = follower.getPose().getHeading();
                    Point currentPos = new Point(follower.getPose().getX(), follower.getPose().getY());
                    Point xVector = new Point(-6, (heading - Math.PI/2), Point.POLAR);

                    Path strafePath = new Path(new BezierLine(
                            currentPos,
                            MathFunctions.addPoints(currentPos, xVector)
                    ));
                    strafePath.setConstantHeadingInterpolation(heading);
                    findNewSample = new SequentialAction(new PedroAction(follower, new PathChain(strafePath), true), new WaitAction(750));
                    findNewSample.init();
                }

                if (!findNewSample.run(t)) {
                    findingNewSample = false;
                    findNewSample = null;
                }

                return true;
            }



        } else {

            //if a position has been chosen, run the entire action until it ends
            if (!totalAction.run(t)) {
                stop(false);

                positionReceived = false;
                return false;
            }

            return true;

        }

    }

    @Override
    public void init() {
        findNewSample = null;

        positionReceived = false;
        findingNewSample = false;
    }

    @Override
    public void stop(boolean interrupted) {
        findNewSample = null;

        positionReceived = false;
        findingNewSample = false;
    }
}
