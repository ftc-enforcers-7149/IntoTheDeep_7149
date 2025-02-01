package org.firstinspires.ftc.teamcode.ActionUtils;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.CombinedTelemetry;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.GlobalData.HardwareConstants;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.LLSampleVision;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.ClawDifferential;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V3Systems.OuttakeExtendo;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Vector;

public class MotionSamplePickupAction extends EventAction {

    private EventAction pickupAction, aboveAction, autoAction, totalAction;

    private LLSampleVision visionSystem;
    private Servo clawWrist;
    private Follower follower;

    private boolean positionReceived;  //true once the robot has a sample it can go after
    private int sampleIndex;  //which sample in the list to go after

    //TODO:
    // Encode x, y, and rot values into one double for limelight results
    // Eg. x = 12.39, y = 3.98, rot = 152.49 --> 11239039815249.0
    // Start with a 1 each time to maintain starting 0
    // then x, y, and rot are encoded in the double

    //TODO:
    // Limelight should properly encode positions and add the 1 at the start
    // Limelight should also order sample info with closest sample first, then second closest, and on

    public MotionSamplePickupAction(LLSampleVision vision, Servo wrist, Follower follow, EventAction aboveSample, EventAction pickupSample) {
        pickupAction = pickupSample;
        aboveAction = aboveSample;

        visionSystem = vision;
        clawWrist = wrist;
        follower = follow;

        autoAction = pickupAction;
        totalAction = pickupAction;
        positionReceived = false;
        sampleIndex = 0;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        //if a sample hasn't been chosen, find one
        if (!positionReceived) {

            double[] sampleInfo = visionSystem.getResultInfo();
            double chosenX = 0.0, chosenY = 0.0, chosenRot = 0.0;

            //go through all the samples until a valid one is found
            for (double pos : sampleInfo) {

                String posStr = pos + "";

                //decode the sample information
                double sampleX = Integer.parseInt(posStr.substring(1, 5)) / 100.0;
                double sampleY = Integer.parseInt(posStr.substring(5, 9)) / 100.0;
                double sampleRot = Integer.parseInt(posStr.substring(9, 14)) / 100.0;

                if (sampleY > HardwareConstants.MAX_EXTENDO_LENGTH) {
                    continue;
                }

                chosenX = sampleX;
                chosenY = sampleY;
                chosenRot = sampleRot;
                positionReceived = true;

            }

            //if a position still hasn't been chosen, terminate the action as no samples are within reach
            if (!positionReceived) {
                return false;
            }

            Point currentPos = new Point(follower.getPose().getX(), follower.getPose().getY());
            Point moveVectorX = new Point(chosenX, follower.getHeadingVector().getTheta() - (Math.PI/2));
            Point moveVectorY = new Point(chosenY - HardwareConstants.MAX_EXTENDO_LENGTH, follower.getHeadingVector().getTheta());
            Point moveVector = MathFunctions.addPoints(moveVectorX, moveVectorY);

            Path moveToPoint = new Path(new BezierLine(
                    currentPos,
                    MathFunctions.addPoints(currentPos, moveVector)
            ));
            autoAction = new PedroAction(follower, new PathChain(moveToPoint), true);

            clawWrist.setPosition(chosenRot / 180.0);
            totalAction = new SequentialAction(
                    new ParallelAction(autoAction, aboveAction),
                    pickupAction);

            totalAction.init();
            return totalAction.run(t);

        } else {

            //if a position has been chosen, run the entire action until it ends
            if (!totalAction.run(t)) {
                stop(false);
                return false;
            }

            return true;

        }

    }

    @Override
    public void init() {
        positionReceived = false;
    }

    @Override
    public void stop(boolean interrupted) {
        positionReceived = false;
    }
}
