package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.PeriodicFollower;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;



@Autonomous(name = "MovementTest")
public class MovementTest extends LinearOpMode {

    ActionManager actionManager;

    Follower follower;

    PedroAction move1, move2;

    Path path1, path2;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //drive = new MecanumPowerDrive(hardwareMap, new Pose2d(-63.25, 15.375, 0), telemetry);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(24, 72, 0));
        PeriodicFollower perFollower = new PeriodicFollower(follower);

        path1 = new Path(
                        new BezierLine(
                                new Point(24, 72),
                                new Point(24, 24)));
        path1.setConstantHeadingInterpolation(0);

        path2 = new Path(
                new BezierCurve(
                        new Point(24, 24),
                        new Point(60, 10),
                        new Point(96, 24)));
        path2.setLinearHeadingInterpolation(0, Math.toRadians(-45));


        move1 = new PedroAction(follower, new PathChain(path1), true);

        move2 = new PedroAction(follower, new PathChain(path2), true);

        actionManager = new ActionManager(this);
        actionManager.attachPeriodicActions(perFollower);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(
                new SequentialAction(

                    move1, move2

                ), (telemetry) -> {}
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
