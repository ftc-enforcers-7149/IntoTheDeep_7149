package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.PedroAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.P2PAction;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.MecanumPowerDrive;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.pathGeneration.Point;

@Disabled
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

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(24, 72, 0));

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


        move1 = new PedroAction(follower, new PathChain(path1), false);

        move2 = new PedroAction(follower, new PathChain(path2), false);

        actionManager = new ActionManager(telemetry, hardwareMap);
        actionManager.attachPeriodicActions();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        //the following shows how actions should be structured and indented

        actionManager.runActionManager(
                new SequentialAction(

                    move1, move2

                )
        );

        telemetry.addData("Finished", "Actions");
        telemetry.update();

    }
}
