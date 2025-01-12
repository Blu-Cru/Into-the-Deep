package org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class SplineRetractCommand extends BoxtubeSplineCommand{
    public SplineRetractCommand(double duration) {
        super(new Pose2d(8.312, 11.723, 1.7), -Math.PI/2, duration);
    }
}
