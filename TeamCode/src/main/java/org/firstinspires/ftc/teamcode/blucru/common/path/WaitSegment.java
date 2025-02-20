package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WaitSegment implements PathSegment{
    final Pose2d pose;
    private final double waitTime;
    private double startTime;

    public WaitSegment(Pose2d pose, double waitTime) {
        this.pose = pose;
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
        return pose;
    }

    @Override
    public boolean failed() {
        return false;
    }

    public void run() {
        Robot.getInstance().dt.pidTo(pose);
    }
}
