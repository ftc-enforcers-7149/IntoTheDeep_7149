package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Used in conjunction with actions that require the position of the robot. Abstracting all
 * localizers under ActionLocalizer allows for the use of many different libraries with
 * this action system.
 * Localizers used with this system must implement this interface and specify the retrieval
 * of the localizer's pose.
 */
public interface ActionLocalizer {

    /**
     * @return The current pose of the localizer
     */
    public Pose2d getLocalizerPose();
}
