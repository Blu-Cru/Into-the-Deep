package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PIDPointSegment implements PathSegment{
    static double FAIL_TIME = 1000;

    Pose2d pose;
    double translationTolerance;
    double startTime;
    boolean stopRequiredToEnd;

    public PIDPointSegment(Pose2d pose, double translationTolerance, boolean stopRequiredToEnd) {
        this.pose = pose;
        this.translationTolerance = translationTolerance;
        this.stopRequiredToEnd = stopRequiredToEnd;
    }

    public PIDPointSegment(Pose2d pose, double translationTolerance) {this(pose, translationTolerance, false);}

    public PIDPointSegment(Pose2d pose, boolean stopRequiredToEnd) {this(pose, 1.5, stopRequiredToEnd);}

    public PIDPointSegment(Pose2d pose) {
        this(pose, 1.5, true);
    }

    public boolean isDone() {
        boolean velSatisfied = !stopRequiredToEnd ||
                Robot.getInstance().dt.vel.vec().norm() < 4.0;

        return Robot.getInstance().dt.inRange(translationTolerance,
                translationTolerance * 0.1)
                && velSatisfied;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTranslationTolerance() {
        return translationTolerance;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void start() {
        startTime = System.currentTimeMillis();
    }

    public boolean failed() {
//        return System.currentTimeMillis() - startTime > FAIL_TIME && Robot.getInstance().drivetrain.isStopped();
        return false;
    }

    public void setTranslationTolerance(double translationTolerance) {
        this.translationTolerance = translationTolerance;
    }
}
