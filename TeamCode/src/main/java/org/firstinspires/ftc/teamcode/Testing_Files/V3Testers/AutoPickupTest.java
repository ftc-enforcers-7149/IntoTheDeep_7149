package org.firstinspires.ftc.teamcode.Testing_Files.V3Testers;

import android.util.EventLog;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ActionManager;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.EventAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.InstantAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.ParallelAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.SequentialAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure.TimedAction;
import org.firstinspires.ftc.teamcode.ActionUtils.ClawRotateAction;
import org.firstinspires.ftc.teamcode.ActionUtils.MotionSamplePickupAction;
import org.firstinspires.ftc.teamcode.GlobalData.HardwareConstants;
import org.firstinspires.ftc.teamcode.Hardware.Chassis.PeriodicFollower;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.FourServoPitchArm;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.LLSampleVision;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "AutoPickup", group = "Testers")
public class AutoPickupTest extends LinearOpMode {

    FourServoPitchArm arm;
    LLSampleVision sampleVision;
    ServoImplEx wristFront, rightExt, leftExt;

    MotionSamplePickupAction pickupAction;
    ActionManager manager;
    Follower follower;
    SequentialAction armPickup;
    ParallelAction armAbove;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = new FourServoPitchArm(this);
        sampleVision = new LLSampleVision(this, 0);
        EventAction clawIntake = new ClawRotateAction(hardwareMap, "frontClaw", 1);
        wristFront = hardwareMap.get(ServoImplEx.class, "frontWrist");
        wristFront.setPwmRange(new PwmControl.PwmRange(500, 2500));

        rightExt = hardwareMap.get(ServoImplEx.class, "extServo");
        rightExt.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        leftExt = hardwareMap.get(ServoImplEx.class, "leftExt");
//        leftExt.setPwmRange(new PwmControl.PwmRange(500, 2500));

        //leftExt.setPosition(0.25);
        rightExt.setPosition(1);
        arm.setPosition(HardwareConstants.PITCH_CAMERA);


        EventAction above = new InstantAction(() -> {
            arm.setPosition(HardwareConstants.PITCH_HOVER - 0.05);
        });

        EventAction pickup = new InstantAction(() -> {
            arm.setPosition(HardwareConstants.PITCH_PICKUP);
        });

        EventAction armUp = new InstantAction(() -> {
            arm.setPosition(HardwareConstants.PITCH_HOVER);
        });

        EventAction extensionOut = new InstantAction(() -> {
            //leftExt.setPosition(1);
            rightExt.setPosition(1);
        });
        EventAction extensionIn = new InstantAction(() -> {
            //leftExt.setPosition(1);
            rightExt.setPosition(0.1);
        });

        armPickup = new SequentialAction(new ParallelAction(pickup, new TimedAction(clawIntake, 1000)), above);
        armAbove = new ParallelAction(above, extensionOut);


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(-90)));

        PeriodicFollower perFollower = new PeriodicFollower(follower);

        manager = new ActionManager(this);
        manager.attachPeriodicActions(perFollower);


        pickupAction = new MotionSamplePickupAction(sampleVision, wristFront, follower, armAbove, armPickup);



        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            double sampleInfo[] = sampleVision.getResultInfo();


            if (sampleInfo[0] != 0) {
                telemetry.addData("Angle", sampleInfo[1]);
                telemetry.addData("X Field", sampleInfo[2]);
                telemetry.addData("Y Field", sampleInfo[3]);
                telemetry.addData("X Cam", sampleInfo[4]);
                telemetry.addData("Y Cam", sampleInfo[5]);

            } else {
                telemetry.addData("No visibile", "sample");
            }

            telemetry.update();

            if (gamepad1.triangle) {
                break;
            }
        }

        manager.runActionManager(
                pickupAction,

                (t) -> {
                    t.addData("Follower heading", follower.getPose().getHeading());
                }
        );

        telemetry.addData("Finished","Actions");
        telemetry.update();

    }
}
