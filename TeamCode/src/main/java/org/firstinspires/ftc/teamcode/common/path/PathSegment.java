package org.firstinspires.ftc.teamcode.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public interface PathSegment {
    boolean isDone();

    void start();

    boolean failed();

    Pose2d getPose();
}
