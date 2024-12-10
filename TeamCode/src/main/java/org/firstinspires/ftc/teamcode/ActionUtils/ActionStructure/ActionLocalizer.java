package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.PathingSystems.pedroPathing.localization.Pose;

public interface ActionLocalizer {

    /**
     * @return The current pose of the localizer
     */
    public Pose2d getLocalizerPose();
}
