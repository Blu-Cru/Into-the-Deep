package org.firstinspires.ftc.teamcode.blucru.common.spline;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class TimedEndHermiteSpline extends EndHermiteSpline {
    double duration, startTime;

    public TimedEndHermiteSpline(Vector2d startPose, Vector2d startVelocity, Vector2d endPose, double duration) {
        super(startPose, startVelocity, endPose);
        this.duration = duration;
    }

    public TimedEndHermiteSpline(Vector2d startPose, Vector2d endPose, double duration) {
        super(startPose, new Vector2d(0, 0), endPose);
        this.duration = duration;
    }

    public TimedEndHermiteSpline start() {
        startTime = System.currentTimeMillis();
        return this;
    }
}
