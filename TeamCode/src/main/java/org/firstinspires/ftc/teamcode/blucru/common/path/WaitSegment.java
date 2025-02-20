package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class WaitSegment implements PathSegment{
    final PathSegment prevSegment;
    private final double waitTime;
    private double startTime;

    public WaitSegment(PathSegment prevSegment, double waitTime) {
        this.prevSegment = prevSegment;
        this.waitTime = waitTime;
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis() - startTime >= waitTime;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }

    public Pose2d getPose() {
        return prevSegment.getPose();
    }

    @Override
    public boolean failed() {
        return false;
    }

    public void run() {
        prevSegment.run();
    }
}
